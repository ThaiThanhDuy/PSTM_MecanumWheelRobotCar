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
float pos1 = 0.0; // Replace with actual position reading
float pos2 = 0.0; // Replace with actual position reading
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
#define GEAR_RATIO 131.0
const int PPR = 11;  // Pulses per revolution of the encoder = 11x131(Gear)
const double R = 0.049;  // Radius (m)
const double C = 2 * 3.14159 * R;  // Circumference (m)


uint32_t currentTime;

typedef struct {
    float position;
    float angularVelocity;
}JointState1;
typedef struct {
    float position;
    float angularVelocity;
}JointState2;
typedef struct {
    float position;
    float angularVelocity;
}JointState3;
typedef struct {
    float position;
    float angularVelocity;
}JointState4;

float angularVelocity1 = 0.0;  // Angular velocity (rad/s)
float linearVelocity1 = 0.0;  // Linear velocity (m/s)
float angular_velocity_deg_s1=0.0; // Angular velocity (deg/s)
float position1 = 0.0;  // Position (m)
float ang1 =0.0;
uint32_t lastTime1=0;
uint32_t lastEncoderCount1=0;
int16_t encoderCount1;
uint32_t currentTime1;

/*JointState1 read1( float* desiredAngularVelocity1,uint32_t time) {
    JointState1 jointState1;

    currentTime1 = time;  // Current time in milliseconds
    uint32_t deltaTime1 = currentTime1 - lastTime1;  // Time since last measurement (ms)

    // Only proceed if at least 100 ms have passed
    if (deltaTime1 >= 100) {

        // Read the encoder count
        int32_t encoderCount1 = Read_Encoder1();  // Read the encoder count

        // Calculate angular velocity (rad/s)
        int16_t deltaCount1 = encoderCount1 - lastEncoderCount1; // New pulses
        if (deltaTime1 > 0) {
            angularVelocity1 = (double)deltaCount1 / PPR * M_PI * 2 / (deltaTime1 / 1000.0);  // rad/s
        } else {
            angularVelocity1 = 0;  // Avoid division by zero
        }
        angular_velocity_deg_s1 = angularVelocity1 * (180.0f / 3.14159f);

        // Calculate linear velocity (m/s)
        linearVelocity1 = angularVelocity1 * R;

        // Calculate position (m)
        position1 = (double)encoderCount1 / PPR * C;
        ang1 += angular_velocity_deg_s1;

        // Update values for the next measurement
        lastEncoderCount1 = encoderCount1;
        lastTime1 = currentTime1; // Update lastTime to the current time

        // Optional: Send joint state or perform other actions
        // sendJointState(position, 0.0, 0.0, 0.0, angularVelocity, 0.0, 0.0, 0.0);
         if(currentTime>=1215){
            Motor_Control1(0, 0); // time error
        }
        controllVel1(controlMotor1(angularVelocity1,*desiredAngularVelocity1,deltaTime1 ));
        jointState1.position = position1;
        jointState1.angularVelocity = angularVelocity1;
    }

    return jointState1;
}*/
JointState1 read1( float* desiredAngularVelocity1,uint32_t time) {
    JointState1 jointState1;

    currentTime1 = time;  // Current time in milliseconds
    uint32_t deltaTime1 = currentTime1 - lastTime1;  // Time since last measurement (ms)

    // Only proceed if at least 100 ms have passed
    if (deltaTime1 >= 100) {

        // Read the encoder count
        int32_t encoderCount1 = Read_Encoder1();  // Read the encoder count

        // Calculate angular velocity (rad/s)
        int16_t deltaCount1 = encoderCount1 - lastEncoderCount1; // New pulses
        int16_t rpm1 = deltaCount1 / PPR;
        if (deltaTime1 > 0) {
            angularVelocity1 =  (double)  rpm1  /  M_PI * 2 / (deltaTime1 / 1000.0);  // rad/s
        } else {
            angularVelocity1 = 0;  // Avoid division by zero
        }
        angular_velocity_deg_s1 = angularVelocity1 * (180.0f / 3.14159f);

        // Calculate linear velocity (m/s)
        linearVelocity1 = angularVelocity1 * R;

        // Calculate position (m)
        position1 = (double)encoderCount1 / PPR * C;
        ang1 += angular_velocity_deg_s1;

        // Update values for the next measurement
        lastEncoderCount1 = encoderCount1;
        lastTime1 = currentTime1; // Update lastTime to the current time

        // Optional: Send joint state or perform other actions
        // sendJointState(position, 0.0, 0.0, 0.0, angularVelocity, 0.0, 0.0, 0.0);
        /* if(currentTime>=1215){
            Motor_Control1(0, 0); // time error
        }*/
        if(angularVelocity1>1){
        	angularVelocity1 = 1;
        }
        else if (angularVelocity1< -1){
        	angularVelocity1 = -1;
        }
        controllVel1(controlMotor1(angularVelocity1,*desiredAngularVelocity1,deltaTime1 ));
        jointState1.position = position1;
        jointState1.angularVelocity = angularVelocity1;
    }

    return jointState1;
}
float angularVelocity2 = 0.0;  // Angular velocity (rad/s)
float linearVelocity2 = 0.0;  // Linear velocity (m/s)
float angular_velocity_deg_s2=0.0; // Angular velocity (deg/s)
float position2 = 0.0;  // Position (m)
float ang2 =0.0;
uint32_t lastTime2=0;
uint32_t lastEncoderCount2=0;
int16_t encoderCount2;
uint32_t currentTime2;

JointState2 read2( float* desiredAngularVelocity2,uint32_t time) {
    JointState2 jointState2;

    currentTime2 = time;  // Current time in milliseconds
    uint32_t deltaTime2 = currentTime2 - lastTime2;  // Time since last measurement (ms)

    // Only proceed if at least 100 ms have passed
    if (deltaTime2 >= 100) {

        // Read the encoder count
        int32_t encoderCount2 = Read_Encoder2();  // Read the encoder count

        // Calculate angular velocity (rad/s)
        int16_t deltaCount2 = encoderCount2 - lastEncoderCount2; // New pulses
        int16_t rpm2 = deltaCount2 / PPR;
        if (deltaTime2 > 0) {
            angularVelocity2 =  (double)  rpm2  /  M_PI * 2 / (deltaTime2 / 1000.0);  // rad/s
        } else {
            angularVelocity2 = 0;  // Avoid division by zero
        }
        angular_velocity_deg_s2 = angularVelocity2 * (180.0f / 3.14159f);

        // Calculate linear velocity (m/s)
        linearVelocity2 = angularVelocity2 * R;

        // Calculate position (m)
        position2 = (double)encoderCount2 / PPR * C;
        ang2 += angular_velocity_deg_s2;

        // Update values for the next measurement
        lastEncoderCount2 = encoderCount2;
        lastTime2 = currentTime2; // Update lastTime to the current time

        // Optional: Send joint state or perform other actions
        // sendJointState(position, 0.0, 0.0, 0.0, angularVelocity, 0.0, 0.0, 0.0);
        /* if(currentTime>=1215){
            Motor_Control1(0, 0); // time error
        }*/
        if(angularVelocity2>1){
                	angularVelocity2 = 1;
                }
                else if (angularVelocity2< -1){
                	angularVelocity2 = -1;
                }
        controllVel2(controlMotor2(angularVelocity2,*desiredAngularVelocity2,deltaTime2 ));
        jointState2.position = position2;
        jointState2.angularVelocity = angularVelocity2;
    }

    return jointState2;
}
float angularVelocity3 = 0.0;  // Angular velocity (rad/s)
float linearVelocity3 = 0.0;  // Linear velocity (m/s)
float angular_velocity_deg_s3=0.0; // Angular velocity (deg/s)
float position3 = 0.0;  // Position (m)
float ang3 =0.0;
uint32_t lastTime3=0;
uint32_t lastEncoderCount3=0;
int16_t encoderCount3;
uint32_t currentTime3;
JointState3 read3( float* desiredAngularVelocity3,uint32_t time) {
    JointState3 jointState3;

    currentTime3 = time;  // Current time in milliseconds
    uint32_t deltaTime3 = currentTime3 - lastTime3;  // Time since last measurement (ms)

    // Only proceed if at least 100 ms have passed
    if (deltaTime3 >= 100) {

        // Read the encoder count
        int32_t encoderCount3 = Read_Encoder3();  // Read the encoder count

        // Calculate angular velocity (rad/s)
        int16_t deltaCount3 = encoderCount3 - lastEncoderCount3; // New pulses
        int16_t rpm3 = deltaCount3 / PPR;
        if (deltaTime3 > 0) {
                  angularVelocity3 =  (double)  rpm3  /  M_PI * 2 / (deltaTime3 / 1000.0);  // rad/s
        } else {
            angularVelocity3 = 0;  // Avoid division by zero
        }
        angular_velocity_deg_s3 = angularVelocity3 * (180.0f / 3.14159f);

        // Calculate linear velocity (m/s)
        linearVelocity3 = angularVelocity3 * R;

        // Calculate position (m)
        position3 = (double)encoderCount3 / PPR * C;
        ang3 += angular_velocity_deg_s3;

        // Update values for the next measurement
        lastEncoderCount3 = encoderCount3;
        lastTime3 = currentTime3; // Update lastTime to the current time

        // Optional: Send joint state or perform other actions
        // sendJointState(position, 0.0, 0.0, 0.0, angularVelocity, 0.0, 0.0, 0.0);
        /* if(currentTime>=1215){
            Motor_Control1(0, 0); // time error
        }*/
        if(angularVelocity3>1){
                	angularVelocity3 = 1;
                }
                else if (angularVelocity3< -1){
                	angularVelocity3 = -1;
                }
        controllVel3(controlMotor3(angularVelocity3,*desiredAngularVelocity3,deltaTime3 ));
        jointState3.position = position3;
        jointState3.angularVelocity = angularVelocity3;
    }

    return jointState3;
}

float angularVelocity4 = 0.0;  // Angular velocity (rad/s)
float linearVelocity4 = 0.0;  // Linear velocity (m/s)
float angular_velocity_deg_s4=0.0; // Angular velocity (deg/s)
float position4 = 0.0;  // Position (m)
float ang4 =0.0;
uint32_t lastTime4=0;
uint32_t lastEncoderCount4=0;
int16_t encoderCount4;
uint32_t currentTime4;
JointState4 read4( float* desiredAngularVelocity4,uint32_t time) {
    JointState4 jointState4;

    currentTime4 = time;  // Current time in milliseconds
    uint32_t deltaTime4 = currentTime4 - lastTime4;  // Time since last measurement (ms)

    // Only proceed if at least 100 ms have passed
    if (deltaTime4 >= 100) {

        // Read the encoder count
        int32_t encoderCount4 = Read_Encoder4();  // Read the encoder count

        // Calculate angular velocity (rad/s)
        int16_t deltaCount4 = encoderCount4 - lastEncoderCount4; // New pulses
        int16_t rpm4 = deltaCount4 / PPR;
        if (deltaTime4 > 0) {
                  angularVelocity4 =  (double)  rpm4  /  M_PI * 2 / (deltaTime4 / 1000.0);  // rad/s
        } else {
            angularVelocity4 = 0;  // Avoid division by zero
        }
        angular_velocity_deg_s4 = angularVelocity4 * (180.0f / 3.14159f);

        // Calculate linear velocity (m/s)
        linearVelocity4 = angularVelocity4 * R;

        // Calculate position (m)
        position4 = (double)encoderCount4 / PPR * C;
        ang4 += angular_velocity_deg_s4;

        // Update values for the next measurement
        lastEncoderCount4 = encoderCount4;
        lastTime4 = currentTime4; // Update lastTime to the current time

        // Optional: Send joint state or perform other actions
        // sendJointState(position, 0.0, 0.0, 0.0, angularVelocity, 0.0, 0.0, 0.0);
        /* if(currentTime>=1215){
            Motor_Control1(0, 0); // time error
        }*/
        if(angularVelocity4>1){
                	angularVelocity4 = 1;
                }
                else if (angularVelocity4< -1){
                	angularVelocity4 = -1;
                }
        controllVel4(controlMotor4(angularVelocity4,*desiredAngularVelocity4,deltaTime4 ));
        jointState4.position = position4;
        jointState4.angularVelocity = angularVelocity4;
    }

    return jointState4;
}
void motor(void){
	  ReadFourFloats(&value1, &value2, &value3, &value4);
	  currentTime = HAL_GetTick();
	  HAL_Delay(1);
	  JointState1 jointState1 = read1(&value1,currentTime);
	  JointState2 jointState2 = read2(&value2,currentTime);
	  JointState3 jointState3 = read3(&value3,currentTime);
	  JointState4 jointState4 = read4(&value4,currentTime);

	  HAL_Delay(1);
	 /* sendJointState(jointState1.position, jointState2.position, jointState3.position, jointState4.position,
	 			  jointState1.angularVelocity,  jointState2.angularVelocity,  jointState3.angularVelocity,  jointState4.angularVelocity);*/
	  sendJointState(jointState1.position,jointState2.position,  jointState3.position, jointState4.position,
 jointState1.angularVelocity, jointState2.angularVelocity,   jointState3.angularVelocity, jointState4.angularVelocity);
	  HAL_Delay(10);
}
