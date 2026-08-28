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
    Sky_Grab_Mode=0,
    Sky_Put_Mode,
    Sky_Carry_Mode//搬运模式，收起机械臂，可用于复位
}Sky_Mode_t;

typedef struct 
{
    bool enable;
    UnitreeMotor* JointGo;//关节电机1
    Zdrive* JointAK;//关节电机2
    Sky_Mode_t Sky_Mode;
    bool FinishFlag;
    uint32_t Go_time;
    bool ResetFlag;
    uint8_t buzzer_phase;
    uint16_t buzzer_timer;
    /* data */
}Sky_t;

extern Sky_t sky;


void Sky_Func(void);
void Sky_Receive(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_Data);
void Sky_Init(void);


#endif
