/*
 * PID.h
 *
 *  Created on: Oct 4, 2023
 *      Author: Quang
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>

typedef struct
{
 float Kp;
 float Ki;
 float Kd;
} PidParameter;

int16_t PID_Calc(PidParameter PID,float speed,float setPoint);

#endif /* INC_PID_H_ */
