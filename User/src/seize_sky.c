#include "seize_sky.h"

#define UnitreeMotor_Use_ID 1
#define ZdriveMotor_Use_ID 1
#define JOINTAK_REDUCTION_RATIO 1
#define SKY_ENABLE 0x01010401
#define SKY_GRAB_FDCANID 0x01010402
#define SKY_PUT_FDCANID 0x01010403
#define SKY_ARM_RESET_FDCANID 0x01010404
#define SKY_ALARM_FDCANID 0x010104EE
#define SKY_RESET_FDCANID 0x010104FF
#define JOINTAK_FINISH_THRESHOLD 0.005f
#define GO_TIME 4000U /* 所有姿态默认运动时间，单位 ms */

/* 每种姿态的目标位置；GO 单位为 rad，AK80 单位与其驱动接口定义一致。 */
typedef struct
{
    float go_position;
    float ak_position_r;
    uint32_t duration_ms;
} SkyPoseConfig_t;

static const SkyPoseConfig_t sky_pose_config[] =
{
    [Sky_Grab_Mode] = {-1.3f, 330.0f, GO_TIME},
    [Sky_Put_Mode] = {-0.7f, 900.0f, GO_TIME},
    [Sky_Carry_Mode] = {-0.64f, 330.0f, GO_TIME},
    [Sky_Silent_Mode] = {-0.64f, 100.0f, GO_TIME}//GO电机以机械限位为0
};

Sky_t sky={0};

/* 初始化轨迹控制器，不会使能电机或发送运动指令。 */
void GoPos_Init(GoPosController_t *ctrl, UnitreeMotor *motor)
{
    if (ctrl == NULL) return;
    ctrl->motor = motor;
    ctrl->start_position = 0.0f;
    ctrl->target_position = 0.0f;
    ctrl->duration_ms = 0U;
    ctrl->elapsed_ms = 0U;
    ctrl->state = GO_POS_IDLE;
}

/* 以当前反馈位置为起点，启动一条五次多项式缓速轨迹。 */
bool GoPos_MoveTo(GoPosController_t *ctrl, float target_position, uint32_t duration_ms)
{
    if (ctrl == NULL || ctrl->motor == NULL || duration_ms == 0U) return false;
    ctrl->start_position = ctrl->motor->data.position;
    ctrl->target_position = target_position;
    ctrl->duration_ms = duration_ms;
    ctrl->elapsed_ms = 0U;
    ctrl->state = GO_POS_RUNNING;
    return true;
}

/* 1 kHz 周期调用：更新目标位置，完成条件仅依据运行时间。 */
void GoPos_Update(GoPosController_t *ctrl, uint32_t delta_ms)
{
    float ratio;
    if (ctrl == NULL || ctrl->motor == NULL || ctrl->state != GO_POS_RUNNING) return;
    if (ctrl->elapsed_ms >= ctrl->duration_ms - ((delta_ms < ctrl->duration_ms) ? delta_ms : ctrl->duration_ms))
        ctrl->elapsed_ms = ctrl->duration_ms;
    else
        ctrl->elapsed_ms += delta_ms;
    ratio = Quintic_Traj((float)ctrl->elapsed_ms, (float)ctrl->duration_ms);
    ctrl->motor->cmd.position = ctrl->start_position +
        (ctrl->target_position - ctrl->start_position) * ratio;
    if (ctrl->elapsed_ms >= ctrl->duration_ms) ctrl->state = GO_POS_FINISHED;
}

bool GoPos_IsFinished(const GoPosController_t *ctrl)
{
    return ctrl != NULL && ctrl->state == GO_POS_FINISHED;
}

bool Sky_Enable(void)
{
    /* 使能只发起静默姿态动作，实际轨迹在 Sky_Func() 中执行。 */
#if USE_UNITREE
    if (sky.JointGo == NULL) return false;
#endif
#if USE_ZMDR
    if (sky.JointAK == NULL) return false;
#endif
#if !(USE_UNITREE || USE_ZMDR)
    return false;
#endif
    sky.enable = true;
#if USE_UNITREE
    sky.JointGo->enable = true;
#endif
#if USE_ZMDR
    sky.JointAK->Begin = true;
    sky.JointAK->mode = Zdrive_Postion;
#endif
    sky.RequestedMode = Sky_Silent_Mode;
    sky.ModeChangePending = true;
    sky.FinishFlag = false;
    return true;
}

void Sky_Disable(void)
{
    sky.enable = false;
#if USE_UNITREE
    if (sky.JointGo != NULL) sky.JointGo->enable = false;
#endif
#if USE_ZMDR
    if (sky.JointAK != NULL) sky.JointAK->mode = Zdrive_Disable;
#endif
#if USE_UNITREE
    sky.GoController.state = GO_POS_IDLE;
#endif
    sky.FinishFlag = true;
}

bool Sky_IsFinished(void)
{
    return sky.FinishFlag;
}

void Sky_Func(void)
{
    bool action_finished = true;

    if (sky.ResetFlag == true)
    {
        __set_FAULTMASK(1);
        NVIC_SystemReset();
    }
    if (!sky.enable) return;

    /* 模式请求由 CAN 回调登记，在定时器控制上下文中统一生效。 */
    if (sky.ModeChangePending)
    {
        Sky_Mode_t mode = sky.RequestedMode;
        sky.ModeChangePending = false;
        if (mode <= Sky_Silent_Mode)
        {
            sky.Sky_Mode = mode;
            sky.FinishFlag = false;
/* 可通过 motor_config.h 单独编译调试 GO 或 AK80。 */
#if USE_UNITREE
            GoPos_MoveTo(&sky.GoController, sky_pose_config[mode].go_position,
                         sky_pose_config[mode].duration_ms);
#endif
#if USE_ZMDR
            sky.JointAK->valSetNow.pos_deg = sky_pose_config[mode].ak_position_r;
#endif
        }
    }

#if USE_UNITREE
    GoPos_Update(&sky.GoController, 1U);
    action_finished = action_finished && GoPos_IsFinished(&sky.GoController);
#endif

#if USE_ZMDR
    action_finished = action_finished &&
        (fabsf(sky.JointAK->valSetNow.pos_deg - sky.JointAK->valReal.pos_deg) <
         JOINTAK_FINISH_THRESHOLD);
#endif

#if USE_UNITREE || USE_ZMDR
    if (action_finished)
        sky.FinishFlag = true;
#else
    (void)action_finished;
#endif
}

void Sky_Receive(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_Data)
{
    FDCAN_TxHeaderTypeDef tx_message = {0};
    uint8_t tx_data[8] = {0};
    tx_message.TxFrameType = FDCAN_DATA_FRAME;
    tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_message.BitRateSwitch = FDCAN_BRS_OFF;
    tx_message.FDFormat = FDCAN_CLASSIC_CAN;
    tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_message.IdType = FDCAN_EXTENDED_ID;
    if (Rxheader.RxFrameType != FDCAN_DATA_FRAME || Rxheader.DataLength < 1 || Rxheader.IdType != FDCAN_EXTENDED_ID) return;

    /* CAN 回调只登记模式请求，不直接操作轨迹控制器。 */
    if (Rxheader.Identifier == SKY_ENABLE && Rxheader.DataLength == 2 && Rx_Data[0] == 'M')
    {
        if (Rx_Data[1]) Sky_Enable(); else if(Rx_Data[1]==0) Sky_Disable();
        tx_message.Identifier = 0x04010101; tx_message.DataLength = 2;
        tx_data[0] = 'M'; tx_data[1] = sky.enable;
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_message, tx_data);
    }
    if (Rxheader.Identifier == SKY_GRAB_FDCANID && Rxheader.DataLength == 2 && Rx_Data[0] == 'G' && Rx_Data[1] == 'S')
    { sky.RequestedMode = Sky_Grab_Mode; sky.ModeChangePending = true; }
    if (Rxheader.Identifier == SKY_PUT_FDCANID && Rxheader.DataLength == 2 && Rx_Data[0] == 'P' && Rx_Data[1] == 'S')
    { sky.RequestedMode = Sky_Put_Mode; sky.ModeChangePending = true; }
    if (Rxheader.Identifier == SKY_ARM_RESET_FDCANID && Rxheader.DataLength == 2 && Rx_Data[0] == 'A' && Rx_Data[1] == 'R')
    { sky.RequestedMode = Sky_Carry_Mode; sky.ModeChangePending = true; }
    if (Rxheader.Identifier == SKY_RESET_FDCANID && Rxheader.DataLength == 2 && Rx_Data[0] == 'R' && Rx_Data[1] == 'S')
    {
        tx_message.Identifier = 0x040101FF; tx_message.DataLength = 2;
        tx_data[0] = 'R'; tx_data[1] = 'S';
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_message, tx_data);
        sky.ResetFlag = true;
    }
    if (Rxheader.Identifier == SKY_ALARM_FDCANID)
    { tx_message.Identifier = 0x040101EE; tx_message.DataLength = 0; HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_message, tx_data); }
}

void Sky_Init(void)
{
    /* 初始化阶段两个电机均失能；GO 保留 set_zero，交由底层完成零点设置。 */
    sky.enable = false;
    sky.ResetFlag = false;
#if USE_UNITREE
    sky.JointGo = &Unitree_motors[UnitreeMotor_Use_ID - 1];
#else
    sky.JointGo = NULL;
#endif
#if USE_ZMDR
    sky.JointAK = &Zmotor[ZdriveMotor_Use_ID - 1];
#else
    sky.JointAK = NULL;
#endif
    sky.Sky_Mode = Sky_Silent_Mode;
    sky.RequestedMode = Sky_Silent_Mode;
    sky.ModeChangePending = false;
#if USE_UNITREE
    sky.JointGo->begin = true;
    sky.JointGo->enable = false;
    sky.JointGo->set_zero = true;
    sky.JointGo->cmd.kp = 0.65f;
    sky.JointGo->cmd.kd = 0.02f;
#endif

    sky.FinishFlag = true;
#if USE_UNITREE
    GoPos_Init(&sky.GoController, sky.JointGo);
#else
    GoPos_Init(&sky.GoController, NULL);
#endif
    Jaw_Init();
#if USE_ZMDR
    sky.JointAK->Begin = false;
    sky.JointAK->mode = Zdrive_Disable;
    sky.JointAK->param.ReductionRatio = JOINTAK_REDUCTION_RATIO;
        if(sky.JointAK->valReal.pos_deg!=0)
    {
        ZdriveSet(0,0x0,Pur);
        sky.initialized=1;
    }
#endif
    if(!USE_ZMDR)
    {
        sky.initialized=1;
    }
}
