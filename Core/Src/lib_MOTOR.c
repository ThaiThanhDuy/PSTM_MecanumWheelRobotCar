/*
 * MOTOR.c
 *
 *  Created on: Nov 27, 2024
 *      Author: Duy
 */


#include "lib_MOTOR.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim12;

extern UART_HandleTypeDef huart2;


int16_t positionMotor1 = 0;
int16_t positionMotor2 = 0;
int16_t positionMotor3 = 0;
int16_t positionMotor4 = 0;

char PM1[10];
char PM2[10];
char PM3[10];
char PM4[10];

int16_t MAX_SPEED = 1000;


// Define constants


// PID control variables


#define MAX_PWM 1000 // Maximum PWM value
#define CPR 2970 // Counts per revolution of the encoder
#define k 0.1 // Motor characteristic constant (RPS per PWM)
// MOTOR
void Motor_Init(void) {

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);  // Init PWM1 -> RPWM
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);  // Init PWM2 -> LPWM

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);  // Init PWM3 -> RPWM
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);  // Init PWM4 -> LPWM

	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);  // Init PWM1 -> RPWM
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);  // Init PWM2 -> LPWM

	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);  // Init PWM1 -> RPWM
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);  // Init PWM2 -> LPWM
}

void Motor_Control1(uint8_t direction, uint16_t speed) {
	// Speed limit -> đến 1000
	if (speed > MAX_SPEED)
		speed = MAX_SPEED;
	// Clockwise
	if (direction == 1) {
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, speed);
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
	// Counter clockwise
	} else if (direction == 2) {
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, speed);

	// Stop
	} else {
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
	}
}
void Motor_Control2(uint8_t direction, uint16_t speed) {

	if (speed > MAX_SPEED)
		speed = MAX_SPEED;
	if (direction == 1) {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, speed);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
	} else if (direction == 2) {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, speed);
	} else {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
	}
}
void Motor_Control3(uint8_t direction, uint16_t speed) {

	if (speed > MAX_SPEED)
		speed = MAX_SPEED;
	if (direction == 1) {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, speed);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	} else if (direction == 2) {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, speed);
	} else {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	}
}
void Motor_Control4(uint8_t direction, uint16_t speed) {

	if (speed > MAX_SPEED)
		speed = MAX_SPEED;
	if (direction == 1) {
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, speed);
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
	} else if (direction == 2) {
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, speed);
	} else {
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
	}
}

void standardMode(int Mode) {
	//Foward
	if (Mode == 1) {
		Motor_Control1(2, 1000);
		Motor_Control2(1, 1000);
		Motor_Control3(2, 1000);
		Motor_Control4(1, 1000);
	}
	//Backward
	if (Mode == 2) {
		Motor_Control1(1, 1000);
		Motor_Control2(2, 1000);
		Motor_Control3(1, 1000);
		Motor_Control4(2, 1000);
	}
	//Sideways Right
	if (Mode == 3) {
		Motor_Control1(2, 1000);
		Motor_Control2(2, 1000);
		Motor_Control3(1, 1000);
		Motor_Control4(1, 1000);
	}
	//Sideways Left
	if (Mode == 4) {
		Motor_Control1(1, 1000);
		Motor_Control2(1, 1000);
		Motor_Control3(2, 1000);
		Motor_Control4(2, 1000);
	}
}
void diagonalMode(int Mode) {
	// Diagonal 45 degree
	if (Mode == 1) {
		Motor_Control1(2, 1000);
		Motor_Control2(0, 0);
		Motor_Control3(0, 0);
		Motor_Control4(1, 1000);
	}
	// Diagonal 135 degree
	if (Mode == 2) {
		Motor_Control1(0, 0);
		Motor_Control2(1, 1000);
		Motor_Control3(2, 1000);
		Motor_Control4(0, 0);
	}
	// Diagonal 225 degree
	if (Mode == 3) {
		Motor_Control1(1, 1000);
		Motor_Control2(0, 0);
		Motor_Control3(0, 0);
		Motor_Control4(2, 1000);
	}
	// Diagonal 315 degree
	if (Mode == 4) {
		Motor_Control1(0, 0);
		Motor_Control2(2, 1000);
		Motor_Control3(1, 1000);
		Motor_Control4(0, 0);
	}
}
void pivotMode(int Mode) {
	// Pivot right forward
	if (Mode == 1) {
		Motor_Control1(2, 1000);
		Motor_Control2(0, 0);
		Motor_Control3(2, 1000);
		Motor_Control4(0, 0);
	}
	// Pivot right backward
	if (Mode == 2) {
		Motor_Control1(1, 1000);
		Motor_Control2(0, 0);
		Motor_Control3(1, 1000);
		Motor_Control4(0, 0);
	}
	// Pivot left forward
	if (Mode == 3) {
		Motor_Control1(0, 0);
		Motor_Control2(1, 1000);
		Motor_Control3(0, 0);
		Motor_Control4(1, 1000);
	}
	// Pivot left backward
	if (Mode == 4) {
		Motor_Control1(0, 0);
		Motor_Control2(2, 1000);
		Motor_Control3(0, 0);
		Motor_Control4(2, 1000);
	}
}
void pivotSidewayMode(int Mode) {
	// Pivot sideway front right
	if (Mode == 1) {
		Motor_Control1(2, 1000);
		Motor_Control2(2, 1000);
		Motor_Control3(0, 0);
		Motor_Control4(0, 0);
	}
	// Pivot sideway front left
	if (Mode == 2) {
		Motor_Control1(1, 1000);
		Motor_Control2(1, 1000);
		Motor_Control3(0, 0);
		Motor_Control4(0, 0);
	}
	// Pivot sideway rear right
	if (Mode == 3) {
		Motor_Control1(0, 0);
		Motor_Control2(0, 0);
		Motor_Control3(1, 1000);
		Motor_Control4(1, 1000);
	}
	// Pivot sideway rear left
	if (Mode == 4) {
		Motor_Control1(0, 0);
		Motor_Control2(0, 0);
		Motor_Control3(2, 1000);
		Motor_Control4(2, 1000);
	}
}
void rotateMode(int Mode) {
	// Rotate Clockwise
	if (Mode == 1) {
		Motor_Control1(2, 1000);
		Motor_Control2(2, 1000);
		Motor_Control3(2, 1000);
		Motor_Control4(2, 1000);
	}
	// Rotate CounterClockwise
	if (Mode == 2) {
		Motor_Control1(1, 1000);
		Motor_Control2(1, 1000);
		Motor_Control3(1, 1000);
		Motor_Control4(1, 1000);
	}
}
void stopMode(void) {

	Motor_Control1(0, 0);
	Motor_Control2(0, 0);
	Motor_Control3(0, 0);
	Motor_Control4(0, 0);

}



// ENCODER


void Encoder_Init(void) {
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); // Init Timer 1 -> mode Encoder
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // Init Timer 3 -> mode Encoder
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL); // Init Timer 5 -> mode Encoder
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL); // Init Timer 8 -> mode Encoder
}
int16_t Read_Encoder1(void) {
	return __HAL_TIM_GET_COUNTER(&htim1);
}
void Reset_Encoder1(void) {
	__HAL_TIM_SET_COUNTER(&htim1, 0);
}
int16_t Read_Encoder2(void) {
	return __HAL_TIM_GET_COUNTER(&htim3);
}
void Reset_Encoder2(void) {
	__HAL_TIM_SET_COUNTER(&htim3, 0);
}
int16_t Read_Encoder3(void) {
	return __HAL_TIM_GET_COUNTER(&htim5);
}
void Reset_Encoder3(void) {
	__HAL_TIM_SET_COUNTER(&htim5, 0);
}
int16_t Read_Encoder4(void) {
	return __HAL_TIM_GET_COUNTER(&htim8);
}
void Reset_Encoder4(void) {
	__HAL_TIM_SET_COUNTER(&htim8, 0);
}

void readEncoder(void) {
	positionMotor1 = Read_Encoder1();
	positionMotor2 = Read_Encoder2();
	positionMotor3 = Read_Encoder3();
	positionMotor4 = Read_Encoder4();
	HAL_Delay(100);
}
//void printEncoder(void) {
//	printf("Encoder 1 : %i\n", positionMotor1);
//	printf("Encoder 2 : %i\n", positionMotor2);
//	printf("Encoder 3 : %i\n", positionMotor3);
//	printf("Encoder 4 : %i\n", positionMotor4);
//	HAL_Delay(100);
//}

// Send data Encoder
/*void sendDataEncoder(void) {
	sprintf(PM1, "EM1: %i\r\n", positionMotor1);
	HAL_UART_Transmit(&huart2, (uint8_t*) PM1, strlen(PM1), HAL_MAX_DELAY);
	sprintf(PM2, "EM2: %i\r\n", positionMotor2);
	HAL_UART_Transmit(&huart2, (uint8_t*) PM2, strlen(PM2), HAL_MAX_DELAY);
	sprintf(PM3, "EM3: %i\r\n", positionMotor3);
	HAL_UART_Transmit(&huart2, (uint8_t*) PM3, strlen(PM3), HAL_MAX_DELAY);
	sprintf(PM4, "EM4: %i\r\n", positionMotor4);
	HAL_UART_Transmit(&huart2, (uint8_t*) PM4, strlen(PM4), HAL_MAX_DELAY);
	HAL_Delay(1000); // Delay for 1 second
}*/


#define DESIRED_RPM 60.0
#define TIME_INTERVAL 1.0 // Time interval for speed measurement in seconds
#define MAX_PWM 1000 // Maximum PWM value
#define KP 100.0 // Proportional gain (tune this value)
#define KI 0.5 // Integral gain (tune this value)
#define KD 1.0 // Derivative gain (tune this value)
float current_rpm = 0.0;
float previous_error = 0.0;
float integral = 0.0;


float Calculate_RPM(uint32_t pulses_counted, float time_interval_seconds) {
    if (time_interval_seconds <= 0) {
        return 0; // Avoid division by zero
    }
    return ((float)pulses_counted / 1441) * (60.0 / time_interval_seconds);
}

void Set_PWM_Duty_Cycle(uint32_t duty_cycle) {
    // Set the PWM duty cycle (implementation depends on your PWM setup)
    Motor_Control1(1, duty_cycle);
}
void run(void){
	uint32_t pulses_counted = positionMotor1;
	        Reset_Encoder1(); // Reset the counter for the next interval

	        // Calculate current RPM
	        current_rpm = Calculate_RPM(pulses_counted, TIME_INTERVAL);
	        printf("%.3f reg/m\n",current_rpm);
	        // PID Control Logic
	        float error = DESIRED_RPM - current_rpm;
	        integral += error * TIME_INTERVAL;
	        float derivative = (error - previous_error) / TIME_INTERVAL;

	        // Calculate PWM value
	        float output = (KP * error) + (KI * integral) + (KD * derivative);
	        if (output > MAX_PWM) output = MAX_PWM; // Limit to max PWM
	        if (output < 0) output = 0; // Ensure PWM is not negative

	        Set_PWM_Duty_Cycle((uint32_t)output); // Set the PWM duty cycle

	        previous_error = error; // Store the error for the next iteration
	        printf("%.3f PWM\n",output);
	        HAL_Delay(TIME_INTERVAL * 1000); // Delay for the time interval
	        }
#define COUNTS_PER_REVOLUTION 1441
#define GEAR_RATIO 131.0
#define DEGREES_PER_REVOLUTION 360.0
#define WHEEL_DIAMETER_MM 96.0  // Wheel diameter in mm
float total_angle;
float Get_Rotation_Angle(void) {
    // Get the current encoder count
    int32_t encoder_count = __HAL_TIM_GET_COUNTER(&htim1);

    // Calculate effective counts per revolution
    float effective_counts_per_revolution = COUNTS_PER_REVOLUTION / GEAR_RATIO;

    // Calculate degrees per count
    float degrees_per_count = DEGREES_PER_REVOLUTION / effective_counts_per_revolution;

    // Calculate the total angle rotated
    return encoder_count * degrees_per_count;
}

// Function to calculate the distance traveled
float Get_Distance_Traveled(void) {
    // Calculate the circumference of the wheel
    float circumference = M_PI * WHEEL_DIAMETER_MM;  // Circumference in mm

    // Get the total rotation angle
    total_angle = Get_Rotation_Angle();

    // Calculate the distance traveled based on the angle
    // Distance = (Angle / 360) * Circumference
    float distance_traveled_mm = (total_angle / DEGREES_PER_REVOLUTION) * circumference;

     // Convert distance from millimeters to meters
     float distance_traveled_m = distance_traveled_mm / 1000.0;

     return distance_traveled_m;  // Distance in meters
}

const int PPR = 1441;  // Pulses per revolution of the encoder
const double R = 0.48;  // Radius (m)
const double C = 2 * 3.14159 * R;  // Circumference (m)
float angularVelocity = 0;  // Angular velocity (rad/s)
float linearVelocity = 0.0;  // Linear velocity (m/s)
float position = 0.0;  // Position (m)
uint32_t lastTime=0;
uint32_t lastEncoderCount=0;
int16_t encoderCount;
float pos1 = 0.0; // Replace with actual position reading
float pos2 = 0.0; // Replace with actual position reading
float pos3 = 0.0; // Replace with actual position reading
float pos4 = 0.0; // Replace with actual position reading
float vel1 = 0.0; // Replace with actual velocity reading
float vel2 = 0.0; // Replace with actual velocity reading
float vel3 = 0.0; // Replace with actual velocity reading
float vel4 = 0.0; // Replace with actual velocity reading

void  read(void){

	   	   	uint32_t currentTime = HAL_GetTick();  // Current time in milliseconds
	        uint32_t deltaTime = currentTime - lastTime;  // Time since last measurement (ms)

	        encoderCount = positionMotor1;  // Read the encoder count

	        // Calculate angular velocity (rad/s)
	        int16_t deltaCount = encoderCount - lastEncoderCount; // New pulses
	        if (deltaTime > 0) {
	            angularVelocity = (double)deltaCount / PPR * 2 * 3.14159 / (deltaTime / 1000.0);  // rad/s
	        } else {
	            angularVelocity = 0;  // Avoid division by zero
	        }

	        // Calculate linear velocity (m/s)
	        linearVelocity = angularVelocity * R;

	        // Calculate position (m)
	        position = (double)encoderCount / PPR * C;

	        // Update values for the next measurement
	        lastEncoderCount = encoderCount;
	        lastTime = currentTime;

	        sendJointState(position, pos2, pos3, pos4, angularVelocity, vel2, vel3, vel4);
	        HAL_Delay(100);  // Delay for readability
}

