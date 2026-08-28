#include "seize_sky.h"

#define UnitreeMotor_Use_ID 1
#define ZdriveMotor_Use_ID 1
/*
SKY_MODE_FDCANID为Sky模式切换的fdCANid，DLC为2，data[0]取值范围为0-3
    Sky_Grab_Mode=0
    Sky_Put_Mode=1
    Sky_Carry_Mode=2

    */

#define SKY_ENABLE 0x01010401
#define SKY_GRAB_FDCANID 0x01010402
#define SKY_PUT_FDCANID 0x01010403
#define SKY_ARM_RESET_FDCANID 0x01010404

#define SKY_ALARM_FDCANID 0x010104EE
#define SKY_RESET_FDCANID 0x010104FF

#define JOINTGO_GRAB_POSITION 1
#define JOINTAK_GRAB_POSITION 1

#define JOINTGO_PUT_POSITION 2
#define JOINTAK_PUT_POSITION 1

#define JOINTGO_CARRY_POSITION 10
#define JOINTAK_CARRY_POSITION 1

#define JOINTGO_FINISH_THRESHOLD 0.5
#define JOINTAK_FINISH_THRESHOLD 0.01

#define GO_TIME 5000//单位为ms

Sky_t sky;


void Sky_Func(void)
{
    static float time;
    if(sky.ResetFlag==true)
    {
        __set_FAULTMASK(1); // 关闭所有的中断，确保执行复位时不被中断打断
        NVIC_SystemReset(); // 系统软件复位，配置好的外设寄存器也一起复位
    }
    if (sky.enable != true)
    {
        return;
    }
    switch (sky.Sky_Mode)
    {

    case Sky_Grab_Mode:
        if(sky.FinishFlag == 0 )
        {
            time=Quintic_Traj(sky.Go_time++,GO_TIME);
            sky.JointGo->cmd.position = JOINTGO_GRAB_POSITION*time;
            sky.JointAK->valSetNow.pos_deg = JOINTAK_GRAB_POSITION;
        }

        break;

    case Sky_Put_Mode:
        if(sky.FinishFlag == 0 )
        {
            time=Quintic_Traj(sky.Go_time++,GO_TIME);
            sky.JointGo->cmd.position = JOINTGO_PUT_POSITION*time;
            sky.JointAK->valSetNow.pos_deg = JOINTAK_PUT_POSITION;
        }
        break;

    case Sky_Carry_Mode:
        if(sky.FinishFlag == 0 )
        {
            time=Quintic_Traj(sky.Go_time++,GO_TIME);
            sky.JointGo->cmd.position = JOINTGO_CARRY_POSITION*time;
            sky.JointAK->valSetNow.pos_deg = JOINTAK_CARRY_POSITION;
        }
        break;
    }
    if (fabs(sky.JointGo->data.position - JOINTGO_CARRY_POSITION) < JOINTGO_FINISH_THRESHOLD &&
        fabs(sky.JointAK->valSetNow.pos_deg - sky.JointAK->valReal.pos_deg) < JOINTAK_FINISH_THRESHOLD)
    {
        sky.FinishFlag = 1;
        sky.Go_time=0;
    }
}
void Sky_Receive(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_Data)
{
    FDCAN_TxHeaderTypeDef tx_message;
    uint8_t tx_data[8];

    if (Rxheader.RxFrameType != FDCAN_DATA_FRAME || Rxheader.DataLength < 1 || Rxheader.IdType != FDCAN_EXTENDED_ID)
    {
        return;
    }

    if (Rxheader.Identifier == SKY_ENABLE && Rxheader.DataLength == 2 && Rx_Data[0] == 'M')
    {
        sky.enable = Rx_Data[1];
        tx_message.TxFrameType = FDCAN_DATA_FRAME;
        tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        tx_message.BitRateSwitch = FDCAN_BRS_OFF;
        tx_message.FDFormat = FDCAN_CLASSIC_CAN;
        tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        tx_message.MessageMarker = 0;
        tx_message.IdType = FDCAN_EXTENDED_ID;
        tx_message.Identifier = 0x04010101;
        tx_message.DataLength = 2;
        tx_data[0] = 'M';
        tx_data[1] = sky.enable;
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_message, tx_data);
    }
    if (Rxheader.Identifier == SKY_GRAB_FDCANID && Rxheader.DataLength == 2 && Rx_Data[0] == 'G' && Rx_Data[1] == 'S')
    {
        sky.Sky_Mode = Sky_Grab_Mode;
        sky.FinishFlag = 0;
    }
    if (Rxheader.Identifier == SKY_PUT_FDCANID && Rxheader.DataLength == 2 && Rx_Data[0] == 'P' && Rx_Data[1] == 'S')
    {
        sky.Sky_Mode = Sky_Put_Mode;
        sky.FinishFlag = 0;
    }
    if (Rxheader.Identifier == SKY_ARM_RESET_FDCANID && Rxheader.DataLength == 2 && Rx_Data[0] == 'A' && Rx_Data[1] == 'R')
    {
        sky.Sky_Mode = Sky_Carry_Mode;
        sky.FinishFlag = 0;
        /* code */
    }
    if (Rxheader.Identifier == SKY_RESET_FDCANID && Rxheader.DataLength == 2 && Rx_Data[0] == 'R' && Rx_Data[1] == 'S')
    {
        tx_message.TxFrameType = FDCAN_DATA_FRAME;
        tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        tx_message.BitRateSwitch = FDCAN_BRS_OFF;
        tx_message.FDFormat = FDCAN_CLASSIC_CAN;
        tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        tx_message.MessageMarker = 0;
        tx_message.IdType = FDCAN_EXTENDED_ID;
        tx_message.Identifier = 0x040101FF;
        tx_message.DataLength = 2;
        tx_data[0] = 'R';
        tx_data[1] = 'S';
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_message, tx_data);

        sky.ResetFlag = true;
    }
    if (Rxheader.Identifier == SKY_ALARM_FDCANID)
    {
        tx_message.TxFrameType = FDCAN_DATA_FRAME;
        tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        tx_message.BitRateSwitch = FDCAN_BRS_OFF;
        tx_message.FDFormat = FDCAN_CLASSIC_CAN;
        tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        tx_message.MessageMarker = 0;
        tx_message.IdType = FDCAN_EXTENDED_ID;
        tx_message.Identifier = 0x040101EE;
        tx_message.DataLength = 0;
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_message, tx_data);
    }
}
void Sky_Init(void)
{
    sky.enable = false;
    sky.ResetFlag=false;
    sky.JointGo = &Unitree_motors[UnitreeMotor_Use_ID - 1];
    sky.JointAK = &Zmotor[ZdriveMotor_Use_ID - 1];
    sky.Sky_Mode = Sky_Carry_Mode;
    sky.Go_time=0;
    /*Unitree Go Motor初始化*/
    sky.JointGo->begin = true;
    sky.JointGo->enable = true;
    sky.JointGo->set_zero = true;

    /*AK-80初始化*/
    sky.JointAK->Begin = true;
    sky.JointAK->mode = Zdrive_Postion;

    sky.FinishFlag = 1;
    Jaw_Init();
}
