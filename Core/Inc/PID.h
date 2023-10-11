/*
 * PID.h
 *
 *  Created on: Oct 4, 2023
 *      Author: Quang
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#define PWM_MAX 999
#define PWM_MIN 0

#include <stdint.h>

typedef struct {
	float Kp;
	float Ki;
	float Kd;
} PidParameter;

int16_t PID_Calc(PidParameter PID, float current, float setPoint);

#endif /* INC_PID_H_ */
