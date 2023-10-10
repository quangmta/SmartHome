/*
 * PID.c
 *
 *  Created on: Oct 4, 2023
 *      Author: Quang
 */
#include "PID.h"
#include "main.h"

#define PWM_MAX 999
#define PWM_MIN 0

uint32_t timerPID_pres=0;
float last_error_pres=0;
float integrated_error_pres=0;
//extern UART_HandleTypeDef huart6;

int16_t PID_Calc(PidParameter PID,float current,float setPoint)
{
	int16_t pidOut=0;
	float pTerm = 0, iTerm = 0,dTerm = 0, dt = 10;
	uint32_t now = HAL_GetTick();
	if (now>timerPID_pres) dt = (float) (HAL_GetTick() - timerPID_pres);
	timerPID_pres= HAL_GetTick();
	float error = setPoint - current;

	pTerm =   PID.Kp  * error;

	integrated_error_pres += error * dt;
	iTerm =   PID.Ki * integrated_error_pres/1000.0;
//	if(iTerm>PWM_MAX/2) iTerm=PWM_MAX/2;
//	else if(iTerm<-PWM_MAX/2) iTerm=-PWM_MAX/2;

	if ( dt != 0) {
		dTerm =  1000*PID.Kd * (error-last_error_pres)/dt;
	}

	pidOut = (int16_t)(pTerm + iTerm + dTerm);

	last_error_pres = error;

	if (pidOut>PWM_MAX) pidOut=PWM_MAX;
	else if(pidOut<PWM_MIN) pidOut=PWM_MIN;
	return pidOut;
}

