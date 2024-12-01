/*
 * MOTOR.h
 *
 *  Created on: Nov 27, 2024
 *      Author: Duy
 */
#include "stm32f4xx_hal.h"
#include "main.h"
#include "string.h"
#include "stdint.h"
#include "stdio.h"

#include "math.h"

#include <lib_UART.h>
//#include <lib_DATA.h>
#ifndef INC_LIB_MOTOR_H_
#define INC_LIB_MOTOR_H_


void Motor_Init(void);
void Motor_Control1(uint8_t direction, uint16_t speed);
void Motor_Control2(uint8_t direction, uint16_t speed);
void Motor_Control3(uint8_t direction, uint16_t speed);
void Motor_Control4(uint8_t direction, uint16_t speed);

void standardMode(int Mode);
void diagonalMode(int Mode);
void pivotMode(int Mode);
void pivotSidewayMode(int Mode);
void rotateMode(int Mode);
void stopMode(void);
void RESET_ALL(void);
void Encoder_Init(void);
int16_t Read_Encoder1(void);
int16_t Read_Encoder2(void);
int16_t Read_Encoder3(void);
int16_t Read_Encoder4(void);
void Reset_Encoder1(void);
void Reset_Encoder2(void);
void Reset_Encoder3(void);
void Reset_Encoder4(void);

void readEncoder(void);
//void sendDataEncoder(void);
void Read_Encoder_Speed();


//void read(void);
float controlMotor1(float velIN,float velTAG,uint32_t deltaTime);
float controlMotor2(float velIN,float velTAG,uint32_t deltaTime);
float controlMotor3(float velIN,float velTAG,uint32_t deltaTime);
float controlMotor4(float velIN,float velTAG,uint32_t deltaTime);
void controllVel1(float signal);
void controllVel2(float signal);
void controllVel3(float signal);
void controllVel4(float signal);

#endif /* INC_MOTOR_H_ */
