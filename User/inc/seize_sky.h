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
    UnitreeMotor *motor;
    float start_position;
    float target_position;
    uint32_t duration_ms;
    uint32_t elapsed_ms;
    GoPosState_t state;
} GoPosController_t;

typedef struct
{
    volatile bool enable;
    UnitreeMotor *JointGo;
    Zdrive *JointAK;
    Sky_Mode_t Sky_Mode;
    volatile Sky_Mode_t RequestedMode;
    volatile bool ModeChangePending;
    volatile bool FinishFlag;
    volatile bool ResetFlag;
    GoPosController_t GoController;
} Sky_t;

extern Sky_t sky;

void Sky_Func(void);
void Sky_Receive(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_Data);
void Sky_Init(void);

/** Initializes the GO motor ramped position controller. */
void GoPos_Init(GoPosController_t *ctrl, UnitreeMotor *motor);
/** Starts a ramped move from the current GO position to a target. */
bool GoPos_MoveTo(GoPosController_t *ctrl, float target_position, uint32_t duration_ms);
/** Updates the GO position trajectory for one control period. */
void GoPos_Update(GoPosController_t *ctrl, uint32_t delta_ms);
/** Returns true when the GO trajectory time has elapsed. */
bool GoPos_IsFinished(const GoPosController_t *ctrl);
/** Enables both joints and requests the silent pose. */
bool Sky_Enable(void);
/** Disables both joints and stops the GO trajectory. */
void Sky_Disable(void);
/** Returns true when the current sky action is complete. */
bool Sky_IsFinished(void);

#endif
