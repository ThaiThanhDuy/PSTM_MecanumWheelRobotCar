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

#ifndef INC_LIB_DATA_H_
#define INC_LIB_DATA_H_

void sendJointState(float pos1, float pos2, float pos3, float pos4, float vel1, float vel2, float vel3, float vel4);
void UART_ReceiveString(uint8_t *buffer, size_t length);
void ReadFourFloats(float *val1, float *val2, float *val3, float *val4);
void resetDataSend(void);
void SR(void);
#endif /* INC_LIB_DATA_H_ */
