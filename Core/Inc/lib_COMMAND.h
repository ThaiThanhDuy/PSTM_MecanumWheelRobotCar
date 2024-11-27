/*
 * lib_COMMAND.h
 *
 *  Created on: Nov 27, 2024
 *      Author: Duy
 */
#include "stm32f4xx_hal.h"
#include "main.h"
#include "string.h"
#include "stdint.h"

#include <lib_UART.h>
#include <lib_MOTOR.h>

#ifndef INC_LIB_COMMAND_H_
#define INC_LIB_COMMAND_H_


void sendDataStatus(uint8_t status);
void reviceCommand(uint8_t command);

#endif /* INC_LIB_COMMAND_H_ */
