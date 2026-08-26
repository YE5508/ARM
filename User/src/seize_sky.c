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

#define SKY_ALARM_FDCANID 0x010204EE
#define SKY_RESET_FDCANID 0x010204FF

#define JOINTGO_GRAB_POSITION 1
#define JOINTAK_GRAB_POSITION 1

#define JOINTGO_PUT_POSITION 2
#define JOINTAK_PUT_POSITION 1

#define JOINTGO_CARRY_POSITION 3
#define JOINTAK_CARRY_POSITION 1

#define JOINTGO_FINISH_THRESHOLD 0.01
#define JOINTAK_FINISH_THRESHOLD 0.01

Sky_t sky;

void Sky_Func(void)
{
    if (sky.enable != true)
    {
        return;
    }
    switch (sky.Sky_Mode)
    {

    case Sky_Grab_Mode:
        sky.JointGo->cmd.position = JOINTGO_GRAB_POSITION;
        sky.JointAK->valSetNow.pos_deg = JOINTAK_GRAB_POSITION;
        break;

    case Sky_Put_Mode:
        sky.JointGo->cmd.position = JOINTGO_PUT_POSITION;
        sky.JointAK->valSetNow.pos_deg = JOINTAK_PUT_POSITION;
        break;

    case Sky_Carry_Mode:
        sky.JointGo->cmd.position = JOINTGO_CARRY_POSITION;
        sky.JointAK->valSetNow.pos_deg = JOINTAK_CARRY_POSITION;
        break;
    }
    if (fabs(sky.JointGo->data.position - sky.JointGo->cmd.position) < JOINTGO_FINISH_THRESHOLD &&
        fabs(sky.JointAK->valSetNow.pos_deg - sky.JointAK->valReal.pos_deg) < JOINTAK_FINISH_THRESHOLD)
    {
        sky.FinishFlag = 1;
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
    if (Rxheader.Identifier == SKY_GRAB_FDCANID)
    {
        sky.Sky_Mode = Sky_Grab_Mode;
        sky.FinishFlag = 0;
    }
    if (Rxheader.Identifier == SKY_PUT_FDCANID)
    {
        sky.Sky_Mode = Sky_Put_Mode;
        sky.FinishFlag = 0;
    }
    if (Rxheader.Identifier == SKY_ARM_RESET_FDCANID)
    {
        sky.Sky_Mode = Sky_Carry_Mode;
        sky.FinishFlag = 0;
        /* code */
    }
    if (Rxheader.Identifier == SKY_RESET_FDCANID)
    {
        Sky_Init();
        BspBuzzer_Alarm(3, 20, 20);
    }
    if (Rxheader.Identifier == SKY_ALARM_FDCANID)
    {
    }
}
void Sky_Init(void)
{
    sky.enable = false;

    sky.JointGo = &Unitree_motors[UnitreeMotor_Use_ID - 1];
    sky.JointAK = &Zmotor[ZdriveMotor_Use_ID - 1];
    sky.Sky_Mode = Sky_Carry_Mode;
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