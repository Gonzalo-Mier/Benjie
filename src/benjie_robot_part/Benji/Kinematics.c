/*
 *  ======= Kinematics ========
 *  Kinematics target-side implementation
 *
 *  Created on: 3/11/2015
 *  Author:     Julio
 */
#include <string.h>
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

#include "Kinematics.h"
#include "Control.h"
//#include "../tcpEchoCC3100.h"

/* include Kinematics internal implementation definitions */
//#include "package/internal/Kinematics.xdc.h"

// External variables
extern int Motor1, Motor2;
extern char datos[TCPPACKETSIZE];
extern int WifiON;
extern uint8_t controlON;

char datoLocal1, datoLocal2;
volatile uint8_t actualiza_ref;
volatile uint8_t mandoON = 0,actualiza_ref = 0;

// Receiving structure
#ifdef FINAL_PROYECT
struct msg_struct tcp_msg;
#endif
#ifdef PWM_TEST
int8_t vDer, vIzq;	// For debugging
#endif
/*
 *  ======== Kinematics_Module_startup ========
 */
Int Kinematics_Module_startup(Int state)
{
	return (Startup_DONE);
}

void Kinematics_Module_Execution( void )
{
	//Semaphore_post(ControlSemaphore);
	Semaphore_pend(StartSemaphore1, BIOS_WAIT_FOREVER);
	while(!WifiON);


	while(1)
	{
		if(dato_rec)
		{
			//System_printf("Comienzo impresion"); // For debugging
			//System_flush();
			Semaphore_pend(WifiSemaphore, BIOS_WAIT_FOREVER);	//Here data must be processed

#ifdef PWM_TEST
			if(datos[0] == '0'){
				_Right_Motor_Speed((float)(int8_t)datos[1]);
				vDer = (int8_t)datos[1];}
			else{
				_Left_Motor_Speed((float)(int8_t)datos[1]);
				vIzq = (int8_t)datos[1];}
#else
			if(datos[0] == 2) // Options change
			{
				Semaphore_pend(ControlSemaphore, BIOS_WAIT_FOREVER);
				controlON = datos[1];
				actualiza_ref = datos[2];
				mandoON = datos[3];
				Semaphore_post(ControlSemaphore);
			}
			else if(datos[0] == 1) // Direct movement order
			{
				if(mandoON){
					Semaphore_pend(ControlSemaphore, BIOS_WAIT_FOREVER);
					_Right_Motor_Speed((float)(int8_t)datos[1]);
					_Left_Motor_Speed((float)(int8_t)datos[2]);
					Semaphore_post(ControlSemaphore);
				}
			}
			else // Camera info
			{
				Semaphore_pend(ControlSemaphore, BIOS_WAIT_FOREVER);
				tcp_msg = *(struct msg_struct *)datos;

				// Store message parameters
				robot.cx = ((double)tcp_msg.robot_marker.cx)/10.0;
				robot.cy = ((double)tcp_msg.robot_marker.cy)/10.0;
				robot.theta = ((double)tcp_msg.robot_marker.theta)/1000.0;
				if (tcp_msg.type == 4)
				{
					ref.cx = ((double)tcp_msg.ref_marker.cx)/10.0;
					ref.cy = ((double)tcp_msg.ref_marker.cy)/10.0;
					ref.theta = ((double)tcp_msg.ref_marker.theta)/1000.0;
					actualiza_ref = 0; // Reference is now actualized
				}
				Semaphore_post(ControlSemaphore);
			}
#endif
			Semaphore_post(WifiSemaphore);
			dato_rec = 0;
		}
	}
}

#ifdef Kinematics_Object
/*
 *  ======== Kinematics_Instance_init ========
 *  Kinematics created or constructed instance object initialization
 */
Void Kinematics_Instance_init(Kinematics_Object *obj, const Kinematics_Params *params)
{
	/* TODO: initialize Kinematics instance state fields */
}
#endif
