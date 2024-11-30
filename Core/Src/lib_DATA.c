/*
 * lib_DATA.c
 *
 *  Created on: Nov 28, 2024
 *      Author: Duy
 */

#include <lib_DATA.h>


extern UART_HandleTypeDef huart2;



char P1[20];
char P2[20];
char P3[20];
char P4[20];
char V1[20];
char V2[20];
char V3[20];
char V4[20];
float pos1 = 1.0; // Replace with actual position reading
float pos2 = -1.0; // Replace with actual position reading
float pos3 = 0.0; // Replace with actual position reading
float pos4 = 0.0; // Replace with actual position reading
float vel1 = 0.0; // Replace with actual velocity reading
float vel2 = 0.0; // Replace with actual velocity reading
float vel3 = 0.0; // Replace with actual velocity reading
float vel4 = 0.0; // Replace with actual velocity reading

/// SEND DATA

void sendJointState(float pos1, float pos2, float pos3, float pos4, float vel1, float vel2, float vel3, float vel4) {
    // Prepare joint state message
    /*sprintf(txBuffer, "pos1:%i vel1:%i pos2:%i vel2:%i pos3:%i vel3:%i pos4:%i vel4:%i\n",
            pos1, vel1, pos2, vel2, pos3, vel3, pos4, vel4);*/

    snprintf(P1, sizeof(P1), "pos1:%.6f ", pos1);
    HAL_UART_Transmit(&huart2, (uint8_t*) P1, strlen(P1), HAL_MAX_DELAY);
    snprintf(V1, sizeof(V1), "vel1:%.6f ", vel1);
    HAL_UART_Transmit(&huart2, (uint8_t*) V1, strlen(V1), HAL_MAX_DELAY);

    snprintf(P2, sizeof(P2), "pos2:%.2f ", pos2);
    HAL_UART_Transmit(&huart2, (uint8_t*) P2, strlen(P2), HAL_MAX_DELAY);
    snprintf(V2, sizeof(V2), "vel2:%.2f ", vel2);
    HAL_UART_Transmit(&huart2, (uint8_t*) V2, strlen(V2), HAL_MAX_DELAY);

    snprintf(P3, sizeof(P3), "pos3:%.2f ", pos3);
    HAL_UART_Transmit(&huart2, (uint8_t*) P3, strlen(P3), HAL_MAX_DELAY);
    snprintf(V3, sizeof(V3), "vel3:%.2f ", vel3);
    HAL_UART_Transmit(&huart2, (uint8_t*) V3, strlen(V3), HAL_MAX_DELAY);

    snprintf(P4, sizeof(P4), "pos4:%.2f ", pos4);
    HAL_UART_Transmit(&huart2, (uint8_t*) P4, strlen(P4), HAL_MAX_DELAY);
    snprintf(V4, sizeof(V4), "vel4:%.2f ", vel4);
    HAL_UART_Transmit(&huart2, (uint8_t*) V4, strlen(V4), HAL_MAX_DELAY);

    HAL_Delay(1000); // Delay for 1 second


}

/// REVICE DATA
float value1, value2, value3, value4;

uint8_t buffer[30]; // Buffer to hold the received string

void UART_ReceiveString(uint8_t *buffer, size_t length) {
    // Clear the buffer before receiving new data
    memset(buffer, 0, length); // Clear the buffer

    // Receive data with a timeout of 256 ms
    if (HAL_UART_Receive(&huart2, buffer, length - 1, 256) == HAL_OK) {
        buffer[length - 1] = '\0'; // Null-terminate the string
    } else {
        // Handle reception error
        printf("UART reception error\n");
    }
}

// Function to read four float values from a received string
void ReadFourFloats(float *val1, float *val2, float *val3, float *val4) {
    HAL_Delay(100); // Wait for 100 ms before receiving new data
    UART_ReceiveString(buffer, sizeof(buffer)); // Receive the string from UART

    // Print the received buffer for debugging
    printf("Received buffer: %s\n", buffer);

    // Pointer to the start of the buffer
    char *start = (char *)buffer;

    // Loop to find and process all valid messages
    while ((start = strstr(start, "c:")) != NULL) {
        // Move the pointer past "c:"
        char *data = start + 2;

        // Find the end of the message (next 'c:' or end of buffer)
        char *end = strstr(data, "c:");
        if (end != NULL) {
            *end = '\0'; // Temporarily terminate the string for parsing
        }

        // Print the data after the prefix for debugging
        printf("Data after prefix: %s\n", data);

        // Parse the string
        char *token = strtok(data, ",");
        if (token != NULL) {
            *val1 = atof(token); // Convert to float and store
            printf("Parsed val1: %.2f\n", *val1);
        }

        token = strtok(NULL, ",");
        if (token != NULL) {
            *val2 = atof(token); // Convert to float and store
            printf("Parsed val2: %.2f\n", *val2);
        }

        token = strtok(NULL, ",");
        if (token != NULL) {
            *val3 = atof(token); // Convert to float and store
            printf("Parsed val3: %.2f\n", *val3);
        }

        token = strtok(NULL, ",");
        if (token != NULL) {
            *val4 = atof(token); // Convert to float and store
            printf("Parsed val4: %.2f\n", *val4);
        }

        // Sum the values
        float sum = *val1 + *val2 + *val3 + *val4;
        printf("Sum of values: %.2f\n", sum);

        // Move the start pointer to the end of the current message for the next iteration
        start = end;
    }
}
/*void UART_ReceiveString(uint8_t *buffer, size_t length) {
    HAL_UART_Receive(&huart2, buffer, length - 1, 256); // Receive data with a timeout of 256 ms
    buffer[length - 1] = '\0'; // Null-terminate the string
    // Clear the buffer before receiving new data
    memset(buffer, 0, length); // Clear the buffer

    // Receive data with a timeout of 256 ms
    if (HAL_UART_Receive(&huart2, buffer, length - 1, 256) == HAL_OK) {
        buffer[length - 1] = '\0'; // Null-terminate the string
    } else {
        // Handle reception error
        printf("UART reception error\n");
    }
}
void ReadFourFloats(float *val1, float *val2, float *val3, float *val4) {
		HAL_Delay(100);
    UART_ReceiveString(buffer, sizeof(buffer)); // Receive the string from UART
    char *endptr; // Pointer to track the end of the parsed string
    char *token = strtok((char *)buffer, ",");


    // Parse the string

    if (token != NULL) *val1 = strtof(token, &endptr); // Convert to float and store

    token = strtok(NULL, ",");
    if (token != NULL) *val2 = strtof(token, &endptr); // Convert to float and store

    token = strtok(NULL, ",");
    if (token != NULL) *val3 = strtof(token, &endptr); // Convert to float and store

    token = strtok(NULL, ",");
    if (token != NULL) *val4 = strtof(token, &endptr); // Convert to float and store

    // Check for negative values

    printf("val1 is: %.2f\n", *val1);
    printf("val2 is: %.2f\n", *val2);
    printf("val3 is: %.2f\n", *val3);
    printf("val4 is: %.2f\n", *val4);


       // Sum the values
       float sum = *val1 + *val2 + *val3 + *val4;
       printf("Sum of values: %.2f\n", sum);

}*/
void resetDataSend(void){
	// Clear the buffer for the next command
	memset(P1, 0, sizeof(P1));
	memset(V1, 0, sizeof(V1));

	memset(P2, 0, sizeof(P2));
	memset(V2, 0, sizeof(V2));

	memset(P3, 0, sizeof(P3));
	memset(V3, 0, sizeof(V3));

	memset(P4, 0, sizeof(P4));
    memset(V4, 0, sizeof(V4));

}
void SR(void){


	ReadFourFloats(&value1, &value2, &value3, &value4);

	//-12.34,56.78,-90.12,34.56 Input example
    //Send back the received values (for verification)
	/*  char output[100];
    snprintf(output, sizeof(output), "Received: %.2f, %.2f, %.2f, %.2f\r\n", value1, value2, value3, value4);
    HAL_UART_Transmit(&huart2, (uint8_t *)output, strlen(output), HAL_MAX_DELAY);*/


    sendJointState(pos1, pos2, pos3, pos4, vel1, vel2, vel3, vel4);

}
