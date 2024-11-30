/*
 * lib_COMMAND.c
 *
 *  Created on: Nov 27, 2024
 *      Author: Duy
 */


#include "lib_COMMAND.h"


int messageIndex = 0;
void sendDataStatus(uint8_t status) {
	const char *messages[] = { "Stop\r\n", "Forward\r\n", "Backward\r\n",
								"Sideways Right\r\n", "Sideways Left\r\n",
								"Pivot right forward\r\n","Pivot right backward\r\n",
								"Pivot left forward\r\n", "Pivot left backward\r\n",
								"Pivot sideway front right\r\n", "Pivot sideway front left",
								"Pivot sideway rear right\r\n", "Pivot sideway rear left\r\n",
								"Rotate Clockwise\r\n", "Rotate CounterClockwise\r\n" };
	if (status == '0') {
		messageIndex = 0;
	} else if (status == '1') {
		messageIndex = 1;
	} else if (status == '2') {
		messageIndex = 2;
	} else if (status == '3') {
		messageIndex = 3;
	} else if (status == '4') {
		messageIndex = 4;
	} else if (status == '5') {
		messageIndex = 0;
	}
	else if (status == '6') {
			messageIndex = 0;
		}
	else if (status == '7') {
			messageIndex = 0;
		}
	else if (status == '8') {
			messageIndex = 0;
		}
	else if (status == '9') {
			messageIndex = 5;
		}
	else if (status == 'A') {
			messageIndex = 6;
		}
	else if (status == 'B') {
			messageIndex = 7;
		}
	else if (status == 'C') {
			messageIndex = 8;
		}
	else if (status == 'D') {
			messageIndex = 9;
		}
	else if (status == 'E') {
			messageIndex = 10;
		}
	else if (status == 'F') {
			messageIndex = 11;
		}
	else if (status == 'G') {
			messageIndex = 12;
		}
	else if (status == 'H') {
			messageIndex = 13;
		}
	else if (status == 'I') {
			messageIndex = 14;
		}

	UART_SendString((char*) messages[messageIndex]);

}

// Just one character
void reviceCommand(uint8_t command) {
	if (command == '0') {
		stopMode();

	} else if (command == '1') {
		standardMode(1);
		HAL_Delay(2000);

	} else if (command == '2') {
		standardMode(2);

	} else if (command == '3') {
		standardMode(3);

	} else if (command == '4') {
		standardMode(4);

	}

	else if (command == '5') {
		diagonalMode(1);
		HAL_Delay(2000);
		stopMode();
	} else if (command == '6') {
		diagonalMode(2);
		HAL_Delay(2000);
		stopMode();
	} else if (command == '7') {
		diagonalMode(3);
		HAL_Delay(2000);
		stopMode();
	} else if (command == '8') {
		diagonalMode(4);
		HAL_Delay(2000);
		stopMode();
	}

	else if (command == '9') {
		pivotMode(1);
	} else if (command == 'A') {
		pivotMode(2);
	} else if (command == 'B') {
		pivotMode(3);
	} else if (command == 'C') {
		pivotMode(4);
	}

	else if (command == 'D') {
		pivotSidewayMode(1);
	} else if (command == 'E') {
		pivotSidewayMode(2);
	} else if (command == 'F') {
		pivotSidewayMode(3);
	} else if (command == 'G') {
		pivotSidewayMode(4);
	}

	else if (command == 'H') {
		rotateMode(1);
	} else if (command == 'I') {
		rotateMode(2);
	}
	else if (command == 's'){
		RESET_ALL();
	}

}
