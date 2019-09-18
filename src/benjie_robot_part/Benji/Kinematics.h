/*
 * Kinematics.h
 *
 *  Created on: 3/11/2015
 *      Author: Julio
 */

#ifndef BENJI_KINEMATICS_H_
#define BENJI_KINEMATICS_H_

//#define PWM_TEST
#define FINAL_PROYECT

#ifndef FINAL_PROYECT
#define TCPPACKETSIZE   2 // Only if messages are only for pwm testing
#define PWM_TEST
#else
#define TCPPACKETSIZE   14	// Classic size for the final proyect
#endif


#ifdef FINAL_PROYECT
struct pos{
	double cx;
	double cy;
	double theta;
};
struct marker{	// Low-size marker structure
	int16_t cx;
	int16_t cy;
	int16_t theta;
};
struct msg_struct{	// Low-size message structure
	uint8_t type;
	uint8_t trash;
	struct marker robot_marker;
	struct marker ref_marker;
};
struct pos robot, ref;
#endif
/* Function declarations */
Int Kinematics_Module_startup(Int state);
void Kinematics_Module_Execution( void );
extern uint8_t dato_rec;

#endif /* BENJI_KINEMATICS_H_ */
