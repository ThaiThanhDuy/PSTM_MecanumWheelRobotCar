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
void RESET_ALL(void){
	Reset_Encoder1();
	Reset_Encoder2();
	Reset_Encoder3();
	Reset_Encoder4();
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



#define GEAR_RATIO 131.0
const int PPR = 1441;  // Pulses per revolution of the encoder = 11x131(Gear)
const double R = 0.049;  // Radius (m)
const double C = 2 * 3.14159 * R;  // Circumference (m)
float angularVelocity = 0.0;  // Angular velocity (rad/s)
float linearVelocity = 0.0;  // Linear velocity (m/s)
float angular_velocity_deg_s=0.0; // Angular velocity (deg/s)
float position = 0.0;  // Position (m)
uint32_t lastTime=0;
uint32_t lastEncoderCount=0;
int16_t encoderCount;
uint32_t currentTime;
#define INTEGRAL_LIMIT 5.0
// PID variables
double targetVelocity = 0.5;  // Desired linear velocity
double error = 0.0;            // Current error
double integral = 0.0;         // Integral of error
double lastError = 0.0;        // Last error for derivative calculation
// PID constants
#define Kp 1.0     // Proportional gain
#define Ki 0.1     // Integral gain
#define Kd 0.01    // Derivative gain
float controlSignal;
float lastControlSignal;

// Kalman filter variables
static double kalmanEstimate = 0.0; // Estimated velocity
static double kalmanErrorCovariance = 1.0; // Error covariance
static const double processNoise = 0.01; // Process noise covariance
static const double measurementNoise = 0.1; // Measurement noise covariance
float ang =0.0;
void controlMotor(float velIN,float velTAG,uint32_t deltaTime) {
	   // Calculate the error
	    error = velTAG - velIN;

	    // Kalman Filter Prediction Step
	    // Predict the next state (velocity)
	    kalmanEstimate = kalmanEstimate; // No control input in this simple example
	    kalmanErrorCovariance += processNoise; // Update error covariance

	    // Kalman Filter Update Step
	    // Update the estimate with the new measurement
	    double kalmanGain = kalmanErrorCovariance / (kalmanErrorCovariance + measurementNoise);
	    kalmanEstimate += kalmanGain * (velIN - kalmanEstimate); // Update estimate
	    kalmanErrorCovariance *= (1 - kalmanGain); // Update error covariance

	    // Calculate the integral
	    integral += error; // Assuming loop runs at a consistent rate
	    if (integral > INTEGRAL_LIMIT) {
	        integral = INTEGRAL_LIMIT; // Clamp the integral term
	    } else if (integral < -INTEGRAL_LIMIT) {
	        integral = -INTEGRAL_LIMIT; // Clamp the integral term
	    }

	    // Calculate the derivative
	    double derivative = (deltaTime > 0) ? (error - lastError) / deltaTime : 0.0;

	    // Apply low-pass filter to the derivative
	    static double filteredDerivative = 0.0; // Static variable for filtering
	    const double alpha = 0.1; // Smoothing factor (0 < alpha < 1)
	    filteredDerivative = alpha * derivative + (1 - alpha) * filteredDerivative;

	    // Calculate the control signal
	    controlSignal = Kp * error + Ki * integral + Kd * filteredDerivative;

	    // Rate limiting the control signal
	    const double MAX_CONTROL_SIGNAL_CHANGE = 0.1; // Maximum change per call
	    if (fabs(controlSignal - lastControlSignal) > MAX_CONTROL_SIGNAL_CHANGE) {
	        if (controlSignal > lastControlSignal) {
	            controlSignal = lastControlSignal + MAX_CONTROL_SIGNAL_CHANGE;
	        } else {
	            controlSignal = lastControlSignal - MAX_CONTROL_SIGNAL_CHANGE;
	        }
	    }


	    // Update last error and control signal
	    lastError = error;
	    lastControlSignal = controlSignal; // Store the last control signal for rate limiting
}

void  read(void){

			currentTime = HAL_GetTick();  // Current time in milliseconds
	        uint32_t deltaTime = currentTime - lastTime;  // Time since last measurement (ms)

	        // Only proceed if at least 100 ms have passed

	               // Read the encoder count
	               int32_t encoderCount = Read_Encoder1();  // Read the encoder count

	               // Calculate angular velocity (rad/s)
	               int16_t deltaCount = encoderCount - lastEncoderCount; // New pulses
	               if (deltaTime > 0) {
	                   angularVelocity = (double)deltaCount / PPR * M_PI * 2 / (deltaTime / 1000.0);  // rad/s
	               } else {
	                   angularVelocity = 0;  // Avoid division by zero
	               }
	               angular_velocity_deg_s = angularVelocity * (180.0f / 3.14159f);

	               // Calculate linear velocity (m/s)
	               linearVelocity = angularVelocity * R;

	               // Calculate position (m)
	               position = (double)encoderCount / PPR * C;
	               ang += angular_velocity_deg_s;
	               // Update values for the next measurement
	               lastEncoderCount = encoderCount;
	               lastTime = currentTime; // Update lastTime to the current time

	               // Optional: Send joint state or perform other actions
	             //  sendJointState(position, 0.0, 0.0, 0.0, angularVelocity, 0.0, 0.0, 0.0);
	              /* if(currentTime>=1215){
	            	   Motor_Control1(0, 0); // time error
	               }*/
	               controlMotor(angularVelocity,0.5,deltaTime );

}



