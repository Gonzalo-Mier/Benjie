/*
 * Control.h
 *
 *  Created on: 3/11/2015
 *      Author: Julio
 */

#ifndef BENJI_CONTROL_H_
#define BENJI_CONTROL_H_

#include <stdint.h>
#include <stdbool.h>
#include <inc/hw_memmap.h>
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/pwm.h"

/* Definitions for easy reversal of the motors */
#define LEFT_MOTOR_REVERSE
#define RIGHT_MOTOR_REVERSE
/* Function declarations */
Int Control_Module_startup(Int state);
void Control_Module_Execution( void );
void PWM_ControlInit( void );
void PWM_ControlEnable( void );

inline void _Left_Motor_Forward(void);
inline void _Left_Motor_Backwards(void);
inline void _Right_Motor_Forward(void);
inline void _Right_Motor_Backwards(void);
inline void _Left_Motor_Halt(void);
inline void _Right_Motor_Halt(void);

void _Right_Motor_Speed(float speed);
void _Left_Motor_Speed(float speed);

extern int WifiON;

#endif /* BENJI_CONTROL_H_ */
