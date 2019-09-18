/*
 *  ======= Control ========
 *  Control target-side implementation
 *
 *  Created on: 3/11/2015
 *  Author:     Julio
 */
#include <xdc/std.h>
#include <xdc/runtime/Startup.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/drivers/GPIO.h>

#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Queue.h>

#include "Control.h"
#include "Kinematics.h"
#include <math.h>

#define PI (3.141592653589793)

/* include Control internal implementation definitions */
//#include "package/internal/Control.xdc.h"
int Motor1, Motor2;
float M1_TOn = 0, M2_TOn = 0, M3_TOn = 0; //1->Left;2->Right;3->Forward wheel


#ifdef FINAL_PROYECT	// Enables the control function

// Constants and variables for the control algorithm
extern struct pos robot, ref;
double errorpos, errorpos1, errorpos2;
double errorang, errorang1, errorang2;
double inc_alpha, inc_M1, inc_M2, inc_v, thetaref;
const double Kp_v = 0.01, Ti_v = 10, Td_v = 0.1, Ts = 0.1;
const double Kp_h = 1, Ti_h = 11, Td_h = 0.1;
double speedD, speedI, speedD1 = 0, speedI1 = 0, speedD2 = 0, speedI2 = 0;

// Control initializer
volatile uint8_t controlON = 0;
#endif
/*
 *  ======== Control_Module_startup ========
 */
Int Control_Module_startup(Int state)
{
	return (Startup_DONE);
}

void Control_Module_Execution( void )
{
	Semaphore_pend(StartSemaphore2, BIOS_WAIT_FOREVER);
	while(!WifiON);
	PWM_ControlInit();
	PWM_ControlEnable();
	_Left_Motor_Halt();
	_Right_Motor_Halt();
	Semaphore_post(StartSemaphore1);
	while(1)
	{
		Task_sleep(1000);	// Do nothing
	}
}
#ifdef FINAL_PROYECT
void Control_interrupt(void)
{
	if(Semaphore_pend(ControlSemaphore, 10/*BIOS_NO_WAIT*/)) // Only if data is ready (10ms timeout)
	{
		if(controlON)
		{
			errorpos = sqrt(pow(ref.cx-robot.cx,2.0)+pow(ref.cy-robot.cy,2.0)); // Error in position
			if(errorpos > 7)	// Minimun error required
			{
				/*
				 * Control algorithm
				 * This algorithm consists on two incremental PID algorithms, one for the orientation and the other
				 * for the position error. All the constants must be pre-declared. This is a classical pure prosecution
				 * control, stated in many books and scientific papers.
				 */
				thetaref = atan2((ref.cy-robot.cy),(ref.cx-robot.cx));
				errorang = thetaref - robot.theta;
				errorang = errorang>0.0?fmod(errorang,(2*PI)):fmod(errorang,(-2*PI));
				errorang = errorang>PI?fmod(errorang,-PI):errorang;
				errorang = errorang<-PI?fmod(errorang,PI):errorang;

				inc_v = Kp_v * (errorpos - errorpos1) + Kp_v * Ts * errorpos / Ti_v + Kp_v * Td_v * (errorpos + errorpos2 - 2 * errorpos1) / Ts;
				inc_alpha = Kp_h * (errorang - errorang1) + Kp_h * Ts * errorang / Ti_h + Kp_h * Td_h * (errorang + errorang2 - 2 * errorang1) / Ts;

				inc_M1 = inc_v-inc_alpha;
				inc_M2 = inc_v+inc_alpha;

				speedI += inc_M1;
				speedD += inc_M2;

				// Speed limiting
				speedI = speedI>100?100:speedI;
				speedI = speedI<-100?-100:speedI;
				speedD = speedD>100?100:speedD;
				speedD = speedD<-100?-100:speedD;

				_Right_Motor_Speed((float)(speedD+speedD1+speedD2)/3); 	// Filtered signal
				_Left_Motor_Speed((float)(speedI+speedI1+speedI2)/3); 	// Filtered signal

				// FOR DEBUGGING
				//System_printf("M1 %d\n",(int)(float)(speedD+speedD1+speedD2)/3);
				//System_printf("M2 %d\n",(int)(float)(speedI+speedI1+speedI2)/3);
				//System_flush();

				// Calculation of future values
				errorpos2 = errorpos1;
				errorpos1 = errorpos;
				errorang2 = errorang1;
				errorang1 = errorang;
				speedD2 = speedD1;
				speedD1 = speedD;
				speedI2 = speedI1;
				speedI1 = speedI;
			}
			else
			{
				_Right_Motor_Speed((float)0.0);	// If error is to little, the car pauses.
				_Left_Motor_Speed((float)0.0);
			}
		}
		Semaphore_post(ControlSemaphore);
	}
}
#endif

void PWM_ControlInit( void )
{
	// Initialize GPIO pins
	GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7 );
	GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_1 | GPIO_PIN_2);

	// Initialize PWM pins
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	GPIOPinConfigure(GPIO_PG1_M0PWM5);	// Forward motor
	GPIOPinConfigure(GPIO_PK4_M0PWM6);	// Left motor
	GPIOPinConfigure(GPIO_PK5_M0PWM7);	// Right motor

	// Initialized PWMs
	PWMGenConfigure(PWM0_BASE, PWM_GEN_0,PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);		//Firstly up, then down
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 120000);											//1KHz

	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, PWMGenPeriodGet(PWM0_BASE, PWM_OUT_5)*M3_TOn ); 	//0% duty cycle
	PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);											//Activate PF0 as output for PWM (not used)
	//1KHz
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, PWMGenPeriodGet(PWM0_BASE, PWM_OUT_6)*M1_TOn ); 	//0% duty cycle
	PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);											//Activate PF1 as output for PWM
	//1KHz
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, PWMGenPeriodGet(PWM0_BASE, PWM_OUT_7)*M2_TOn ); 	//0% duty cycle
	PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);											//Activate PF2 as output for PWM

	GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_7,GPIO_PIN_7);
}

void PWM_ControlEnable( void )
{
	// This function initializes the PWM interrupts
	PWMGenEnable(PWM0_BASE, PWM_GEN_0);
	IntMasterEnable();
}

void _Right_Motor_Speed(float speed)
{
	speed = speed>100?100:speed;	// Treshold
	speed = speed<-100?100:speed;	// Treshold
	if(speed < 10 && speed > -10) 	// Too little speed (< 10% DC)
	{
		// Halt motor and PWM
		M2_TOn = 0;
		_Right_Motor_Halt();
		PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, false);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, PWMGenPeriodGet(PWM0_BASE, PWM_OUT_7)*M2_TOn );
	}
	else if(speed > 0)	// Go forward
	{
		M2_TOn = speed/100;
		_Right_Motor_Forward();
		PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, PWMGenPeriodGet(PWM0_BASE, PWM_OUT_7)*M2_TOn );
	}
	else
	{
		M2_TOn = speed/100;
		_Right_Motor_Backwards();
		PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, PWMGenPeriodGet(PWM0_BASE, PWM_OUT_7)*M2_TOn );
	}
}

void _Left_Motor_Speed(float speed)
{
	//speedI = speed;
	speed = speed>100?100:speed;
	speed = speed<-100?100:speed;
	if(speed < 10 && speed > -10)	// Too little speed (< 10% DC)
	{
		M1_TOn = 0;
		_Left_Motor_Halt();
		PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, false);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, PWMGenPeriodGet(PWM0_BASE, PWM_OUT_6)*M1_TOn );
	}
	else if(speed > 0)	// Go forward
	{
		M1_TOn = speed/100;
		_Left_Motor_Forward();
		PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, PWMGenPeriodGet(PWM0_BASE, PWM_OUT_6)*M1_TOn );
	}
	else
	{
		M1_TOn = speed/100;
		_Left_Motor_Backwards();
		PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, PWMGenPeriodGet(PWM0_BASE, PWM_OUT_6)*M1_TOn );
	}
}

void _Left_Motor_Halt(void)
{
	GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_2 | GPIO_PIN_1,GPIO_PIN_2 | GPIO_PIN_1);	// Both H-Bridge pins to HIGH
}

void _Right_Motor_Halt(void)
{
	GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_2 | GPIO_PIN_1,GPIO_PIN_2 | GPIO_PIN_1);	// Both H-Bridge pins to HIGH
}

void _Left_Motor_Forward(void)
{
#ifdef LEFT_MOTOR_REVERSE	// Can be easily reversed
	GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_2 | GPIO_PIN_1,GPIO_PIN_2);	// Needed H-Bridge Pin to HIGH
#else
	GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_2 | GPIO_PIN_1,GPIO_PIN_1);
#endif
}

void _Left_Motor_Backwards(void)
{
#ifdef LEFT_MOTOR_REVERSE	// Can be easily reversed
	GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_2 | GPIO_PIN_1,GPIO_PIN_1);	// Needed H-Bridge Pin to HIGH
#else
	GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_2 | GPIO_PIN_1,GPIO_PIN_2);
#endif
}

void _Right_Motor_Forward(void)
{
#ifdef RIGHT_MOTOR_REVERSE	// Can be easily reversed
	GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_2 | GPIO_PIN_1,GPIO_PIN_2);	// Needed H-Bridge Pin to HIGH
#else
	GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_2 | GPIO_PIN_1,GPIO_PIN_1);
#endif
}

void _Right_Motor_Backwards(void)
{
#ifdef RIGHT_MOTOR_REVERSE	// Can be easily reversed
	GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_2 | GPIO_PIN_1,GPIO_PIN_1);	// Needed H-Bridge Pin to HIGH
#else
	GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_2 | GPIO_PIN_1,GPIO_PIN_2);
#endif
}

#ifdef Control_Object
/*
 *  ======== Control_Instance_init ========
 *  Control created or constructed instance object initialization
 */
Void Control_Instance_init(Control_Object *obj, const Control_Params *params)
{
	/* TODO: initialize Control instance state fields */
}
#endif
