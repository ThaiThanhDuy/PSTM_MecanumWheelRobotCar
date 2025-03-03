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
#include <stdbool.h>
#ifndef INC_LIB_DATA_H_
#define INC_LIB_DATA_H_

extern I2C_HandleTypeDef hi2c1;




float moving_average_filter1(float new_velocity);
float calculate_pwm1(float desired_velocity);
float PID_Controller1(float Kp, float Ki, float Kd, float *integral,float last_error, float setpoint, float measured_value);

float moving_average_filter2(float new_velocity);
float calculate_pwm2(float desired_velocity);
float PID_Controller2(float Kp, float Ki, float Kd, float *integral,float last_error, float setpoint, float measured_value);

float moving_average_filter3(float new_velocity);
float calculate_pwm3(float desired_velocity);
float PID_Controller3(float Kp, float Ki, float Kd, float *integral,float last_error, float setpoint, float measured_value);

float moving_average_filter4(float new_velocity);
float calculate_pwm4(float desired_velocity);
float PID_Controller4(float Kp, float Ki, float Kd, float *integral,float last_error, float setpoint, float measured_value);


//void sendJointState(float lx,float ly,float az, float yaw);
void sendJointState(float lx, float ly, float az, float yaw,float p1,float p2,float p3,float p4);
void UART_ReceiveString(uint8_t *buffer, size_t length);
void ReadFourFloats(float *linearX, float *linearY, float *angularZ);

void BNO055_Init(I2C_HandleTypeDef *hi2c);
void BNO055_Read_Euler_Angles(I2C_HandleTypeDef *hi2c, float *yaw, float *pitch, float *roll);
void readBNO055(void);


void Forward_kinematic_car(float linear_x,float linear_y,float angular_z);
void Inverse_kinematic_car(float v1, float v2, float v3, float v4);

void motor(void);
float PID_ControllerX(float Kp, float Ki, float Kd, float *integral,
		float last_error, float setpoint, float measured_value);
float PID_ControllerY(float Kp, float Ki, float Kd, float *integral,
		float last_error, float setpoint, float measured_value);
float PID_ControllerZ(float Kp, float Ki, float Kd, float *integral,
		float last_error, float setpoint, float measured_value);
#endif /* INC_LIB_DATA_H_ */
