/*
 * UART.c
 *
 *  Created on: Nov 27, 2024
 *      Author: Duy
 */

#include "lib_UART.h"


extern UART_HandleTypeDef huart2;

void UART_SendString(char *str) {
	HAL_UART_Transmit(&huart2, (uint8_t*) str, strlen(str), HAL_MAX_DELAY);
}

void UART_ReceiveData(void) {
	char receivedData[100]; // Buffer to hold received data
	HAL_UART_Receive(&huart2, (uint8_t*) receivedData, sizeof(receivedData) - 1,
	HAL_MAX_DELAY); // Receive data

	// Echo back the received data
	UART_SendString("Received: ");
	UART_SendString(receivedData);
	UART_SendString("\n");
}


