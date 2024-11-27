/*
 * UART.h
 *
 *  Created on: Nov 27, 2024
 *      Author: Duy
 */
//Header Files
#include "stm32f4xx_hal.h"
#include "main.h"
#include "string.h"
#include "stdint.h"


#ifndef INC_LIB_UART_H_
#define INC_LIB_UART_H_

void UART_SendString(char *str);
void UART_ReceiveData(void);


#endif /* INC_UART_H_ */
