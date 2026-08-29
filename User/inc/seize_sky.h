#ifndef SEIZE_SKY_H
#define SEIZE_SKY_H

#include "includes.h"
#include "UnitreeMotor.h"
#include "ZDrive.h"
#include "holding_jaw.h"
#include "bsp_buzzer.h"
#include "stm32h7xx.h"

typedef enum
{
    /* 工作姿态编号与 CAN 控制协议保持一致，勿随意调整前三项数值。 */
    Sky_Grab_Mode = 0,
    Sky_Put_Mode,
    Sky_Carry_Mode,
    Sky_Silent_Mode
} Sky_Mode_t;

typedef enum
{
    GO_POS_IDLE = 0,
    GO_POS_RUNNING,
    GO_POS_FINISHED,
    GO_POS_ERROR
} GoPosState_t;

typedef struct
{
    UnitreeMotor *motor;       /* 绑定的 GO 电机对象 */
    float start_position;      /* 本次轨迹起点，单位 rad */
    float target_position;     /* 本次轨迹终点，单位 rad */
    uint32_t duration_ms;      /* 轨迹总时间，单位 ms */
    uint32_t elapsed_ms;       /* 已运行时间，单位 ms */
    GoPosState_t state;        /* 当前轨迹状态 */
} GoPosController_t;

typedef struct
{
    volatile bool enable;                    /* 总使能状态 */
    UnitreeMotor *JointGo;                   /* GO 电机对象 */
    Zdrive *JointAK;                         /* AK80 电机对象 */
    Sky_Mode_t Sky_Mode;                     /* 当前已生效模式 */
    volatile Sky_Mode_t RequestedMode;       /* CAN 请求的待切换模式 */
    volatile bool ModeChangePending;         /* 有新模式待处理 */
    volatile bool FinishFlag;                /* 当前动作完成标志 */
    volatile bool ResetFlag;                 /* 请求系统复位 */
    GoPosController_t GoController;          /* GO 缓速位控控制器 */
} Sky_t;

extern Sky_t sky;

/** 在 1 kHz 定时器控制周期中执行模式切换和电机轨迹更新。 */
void Sky_Func(void);
/** 解析天空机械臂的 CAN 控制命令并登记控制请求。 */
void Sky_Receive(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_Data);
/** 初始化天空机械臂状态、电机对象和 GO 轨迹控制器。 */
void Sky_Init(void);

/** 初始化 GO 电机缓速位置控制器。 */
void GoPos_Init(GoPosController_t *ctrl, UnitreeMotor *motor);
/** 从 GO 当前反馈位置启动到目标位置的缓速运动。 */
bool GoPos_MoveTo(GoPosController_t *ctrl, float target_position, uint32_t duration_ms);
/** 按一个控制周期更新 GO 位置轨迹。 */
void GoPos_Update(GoPosController_t *ctrl, uint32_t delta_ms);
/** 查询 GO 轨迹运行时间是否结束。 */
bool GoPos_IsFinished(const GoPosController_t *ctrl);
/** 使能两个关节电机，并请求进入静默姿态。 */
bool Sky_Enable(void);
/** 失能两个关节电机，并停止 GO 轨迹。 */
void Sky_Disable(void);
/** 查询当前天空机械臂动作是否完成。 */
bool Sky_IsFinished(void);

#endif
