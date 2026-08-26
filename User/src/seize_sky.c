#include "seize_sky.h"

#define UnitreeMotor_Use_ID 1
#define ZdriveMotor_Use_ID 1
/*
SKY_MODE_FDCANID为Sky模式切换的fdCANid，DLC为2，data[0]取值范围为0-3
    Sky_Idle_Mode=0,
    Sky_Grab_Mode=1
    Sky_Put_Mode=2
    Sky_Carry_Mode=3

    */

#define SKY_MODE_FDCANID 0x010203E2
#define JOINTGO_GRAB_POSITION 1
#define JOINTAK_GRAB_POSITION 1

#define JOINTGO_PUT_POSITION 2
#define JOINTAK_PUT_POSITION 1

#define JOINTGO_CARRY_POSITION 3
#define JOINTAK_CARRY_POSITION 1


Sky_t sky;



void Sky_Func(void)
{
    if(sky.enable!=true)
    {
        return;
    }
    switch (sky.Sky_Mode)
    {

    case Sky_Grab_Mode:
        sky.JointGo->cmd.position=JOINTGO_GRAB_POSITION;
        sky.JointAK->valSetNow.pos_deg=JOINTAK_GRAB_POSITION;
        break;

    case Sky_Put_Mode:
        sky.JointGo->cmd.position=JOINTGO_PUT_POSITION;
        sky.JointAK->valSetNow.pos_deg=JOINTAK_PUT_POSITION;
        break;

    case Sky_Carry_Mode:
        sky.JointGo->cmd.position=JOINTGO_CARRY_POSITION;
        sky.JointAK->valSetNow.pos_deg=JOINTAK_CARRY_POSITION;
        break;    
    
    }
}
void Sky_Receive(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_Data)
{
    if (Rxheader.RxFrameType!=FDCAN_DATA_FRAME||Rxheader.DataLength<1||Rxheader.IdType!=FDCAN_EXTENDED_ID)
    {
        return;
    }

    if(Rxheader.Identifier == SKY_MODE_FDCANID)
    {
        sky.Sky_Mode = Rx_Data[0];
    }
}
void Sky_Init(void)
{
    sky.enable =false;

    sky.JointGo = &Unitree_motors[UnitreeMotor_Use_ID-1];
    sky.JointAK = &Zmotor[ZdriveMotor_Use_ID-1];
    sky.Sky_Mode = Sky_Carry_Mode;
     /*Unitree Go Motor初始化*/
    sky.JointGo->begin = true;
    sky.JointGo->enable = true;
    sky.JointGo->set_zero =true;

    /*AK-80初始化*/
    sky.JointAK->Begin=true;
    sky.JointAK->mode=Zdrive_Postion;
    Jaw_Init();
}