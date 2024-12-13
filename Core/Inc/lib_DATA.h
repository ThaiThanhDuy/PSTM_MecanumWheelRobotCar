/*
 * lib_DATA.h
 *
 *  Created on: Nov 28, 2024
 *      Author: Duy
 */
#include "stm32f4xx_hal.h"
#include "main.h"
#include "string.h"
#include "stdint.h"
#include "stdlib.h"
#include  "stdio.h"
#include "math.h"
#include <lib_MOTOR.h>
#ifndef INC_LIB_DATA_H_
#define INC_LIB_DATA_H_


void resetDataSend(void);
void SR(void);
void calculateVel1(float velTag, float current_time);
void reset_system();
void motor(void);
void UART_ReceiveString(uint8_t *buffer, size_t length);
void ReadFourFloats(float *val1, float *val2, float *val3, float *val4);
float moving_average_filter(float new_velocity);
float calculate_pwm(float desired_velocity);
float PID_Controller(float Kp, float Ki, float Kd, float *integral, float last_error, float setpoint, float measured_value);
void sendJointState(float pos1, float pos2, float pos3, float pos4, float velO1, float velO2, float velO3, float velO4);




#endif /* INC_LIB_DATA_H_ */
