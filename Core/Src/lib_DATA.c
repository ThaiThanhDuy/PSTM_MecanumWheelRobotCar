/*
 * lib_DATA.c
 *
 *  Created on: Nov 28, 2024
 *      Author: Duy
 */

#include <lib_DATA.h>
#include <stddef.h>
#include <stm32f4xx_hal_def.h>
#include <stm32f4xx_hal_uart.h>
#include <sys/_stdint.h>
#include "stm32f4xx_it.h"

extern UART_HandleTypeDef huart2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim12;

extern I2C_HandleTypeDef hi2c1;
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
float velR1 = 0.0; // Replace with actual velocity reading
float velR2 = 0.0; // Replace with actual velocity reading
float velR3 = 0.0; // Replace with actual velocity reading
float velR4 = 0.0; // Replace with actual velocity reading


// BNO055 I2C address
#define BNO055_ADDRESS 0x28 << 1 // BNO055 I2C address (shifted for HAL)

#define BNO055_CHIP_ID 0x00
#define BNO055_OPR_MODE 0x3D
#define BNO055_EULER_H 0x1A

float yaw, pitch, roll;
float yaw_filtered = 0.0;
float pitch_filtered = 0.0;
float roll_filtered = 0.0;
const float alpha = 0.1; // Filter coefficient

// Quaternion components
float qx, qy, qz, qw;
float qx_filtered = 0.0, qy_filtered = 0.0, qz_filtered = 0.0, qw_filtered = 0.0;
const float q_alpha = 0.1; // Filter coefficient for quaternions
void BNO055_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t chip_id;
    HAL_I2C_Mem_Read(hi2c, BNO055_ADDRESS, BNO055_CHIP_ID, 1, &chip_id, 1, HAL_MAX_DELAY);

    if (chip_id != 0xA0) {
        // Handle error: BNO055 not found
    }

    // Set the operation mode to NDOF
    uint8_t mode = 0x0C; // NDOF mode
    HAL_I2C_Mem_Write(hi2c, BNO055_ADDRESS, BNO055_OPR_MODE, 1, &mode, 1, HAL_MAX_DELAY);
    HAL_Delay(20); // Wait for the mode to change
}

void BNO055_Read_Euler_Angles(I2C_HandleTypeDef *hi2c, float *yaw, float *pitch, float *roll) {
    uint8_t data[6];
    HAL_I2C_Mem_Read(hi2c, BNO055_ADDRESS, BNO055_EULER_H, 1, data, 6, HAL_MAX_DELAY);

    // Convert the data to float values
    *yaw = (float)((data[0] | (data[1] << 8)) / 16.0);
    *pitch = (float)((data[2] | (data[3] << 8)) / 16.0);
    *roll = (float)((data[4] | (data[5] << 8)) / 16.0);
}

void calculateQuaternion(float yaw, float pitch, float roll) {
    // Convert degrees to radians
    float yaw_rad = yaw * (M_PI / 180.0);
    float pitch_rad = pitch * (M_PI / 180.0);
    float roll_rad = roll * (M_PI / 180.0);

    // Calculate quaternion components
    qw = cos(yaw_rad / 2) * cos(pitch_rad / 2) * cos(roll_rad / 2) + sin(yaw_rad / 2) * sin(pitch_rad / 2) * sin(roll_rad / 2);
    qx = sin(yaw_rad / 2) * cos(pitch_rad / 2) * cos(roll_rad / 2) - cos(yaw_rad / 2) * sin(pitch_rad / 2) * sin(roll_rad / 2);
    qy = cos(yaw_rad / 2) * sin(pitch_rad / 2) * cos(roll_rad / 2) + sin(yaw_rad / 2) * cos(pitch_rad / 2) * sin(roll_rad / 2);
    qz = cos(yaw_rad / 2) * cos(pitch_rad / 2) * sin(roll_rad / 2) - sin(yaw_rad / 2) * sin(pitch_rad / 2) * cos(roll_rad / 2);
}

void readBNO055(void) {
    // Read raw Euler angles from the BNO055
    BNO055_Read_Euler_Angles(&hi2c1, &yaw, &pitch, &roll);

    // Normalize yaw to be within 0 to 360 degrees
  //  yaw = fmod(yaw, 360.0);


    // Convert yaw from 0-360 to 360-0
       yaw = 360.0 - yaw;
       if (yaw < 0) {
               yaw += 360.0; // Wrap around if negative
           }
    // Apply low-pass filter to yaw, roll, and pitch
    yaw_filtered = alpha * yaw + (1 - alpha) * yaw_filtered;
    roll_filtered = alpha * roll + (1 - alpha) * roll_filtered;
    pitch_filtered = alpha * pitch + (1 - alpha) * pitch_filtered;

    // Calculate quaternion from filtered Euler angles
    calculateQuaternion(yaw_filtered, pitch_filtered, roll_filtered);
    float norm = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
     if (norm > 0.0) {
         qw /= norm;
         qx /= norm;
         qy /= norm;
         qz /= norm;
     }
     // Apply low-pass filter to quaternion components
         qx_filtered = q_alpha * qx + (1 - q_alpha) * qx_filtered;
         qy_filtered = q_alpha * qy + (1 - q_alpha) * qy_filtered;
         qz_filtered = q_alpha * qz + (1 - q_alpha) * qz_filtered;
         qw_filtered = q_alpha * qw + (1 - q_alpha) * qw_filtered;
    // Output the filtered values and quaternion
   /* printf("Filtered Yaw: %.2f\n", yaw_filtered);
    printf("Filtered Roll: %.2f\n", roll_filtered);
    printf("Filtered Pitch: %.2f\n", pitch_filtered);
    printf("Quaternion: qx: %.4f, qy: %.4f, qz: %.4f, qw: %.4f\n", qx, qy, qz, qw);*/
}

uint8_t buffer[30]; // Buffer to hold the received string
float value1, value2, value3, value4;
float oldValue1, oldValue2, oldValue3, oldValue4;
float newValue1, newValue2, newValue3, newValue4; // Store new values
/// SEND DATA

char txBuffer[256];
void sendJointState(float pos1, float pos2, float pos3, float pos4, float velO1,
		float velO2, float velO3, float velO4, float yaw) {
	// Prepare joint state message
	/*sprintf(txBuffer, "pos1:%i vel1:%i pos2:%i vel2:%i pos3:%i vel3:%i pos4:%i vel4:%i\n",
	 pos1, vel1, pos2, vel2, pos3, vel3, pos4, vel4);

	  snprintf(P1, sizeof(P1), "pos1:%.2f ", pos1);
	 HAL_UART_Transmit(&huart2, (uint8_t*) P1, strlen(P1), HAL_MAX_DELAY);
	 snprintf(V1, sizeof(V1), "vel1:%.2f ", velO1);
	 HAL_UART_Transmit(&huart2, (uint8_t*) V1, strlen(V1), HAL_MAX_DELAY);

	 snprintf(P2, sizeof(P2), "pos2:%.2f ", pos2);
	 HAL_UART_Transmit(&huart2, (uint8_t*) P2, strlen(P2), HAL_MAX_DELAY);
	 snprintf(V2, sizeof(V2), "vel2:%.2f ", velO2);
	 HAL_UART_Transmit(&huart2, (uint8_t*) V2, strlen(V2), HAL_MAX_DELAY);

	 snprintf(P3, sizeof(P3), "pos3:%.2f ", pos3);
	 HAL_UART_Transmit(&huart2, (uint8_t*) P3, strlen(P3), HAL_MAX_DELAY);
	 snprintf(V3, sizeof(V3), "vel3:%.2f ", velO3);
	 HAL_UART_Transmit(&huart2, (uint8_t*) V3, strlen(V3), HAL_MAX_DELAY);

	 snprintf(P4, sizeof(P4), "pos4:%.2f ", pos4);
	 HAL_UART_Transmit(&huart2, (uint8_t*) P4, strlen(P4), HAL_MAX_DELAY);
	 snprintf(V4, sizeof(V4), "vel4:%.2f ", velO4);
	 HAL_UART_Transmit(&huart2, (uint8_t*) V4, strlen(V4), HAL_MAX_DELAY);

	 HAL_Delay(100);
*/

	// Format the joint state message into the buffer
	snprintf(txBuffer, sizeof(txBuffer),
			"pos1:%.2f vel1:%.2f pos2:%.2f vel2:%.2f pos3:%.2f vel3:%.2f pos4:%.2f vel4:%.2f yaw:%.2f\n",
			pos1, velO1, pos2, velO2, pos3, velO3, pos4, velO4,yaw);

	// Transmit the complete message over UART
	HAL_UART_Transmit(&huart2, (uint8_t*) txBuffer, strlen(txBuffer),
			HAL_MAX_DELAY);
	// Optional delay to prevent flooding the UART
	HAL_Delay(100);
}



/*
void sendJointState(float pos1, float pos2, float pos3, float pos4,
                    float velO1, float velO2, float velO3, float velO4, float yaw) {
    // Prepare a buffer to hold the complete joint state message
    // Ensure this buffer is large enough to hold the entire message

    // Format the joint state message into the buffer
    int length = snprintf(txBuffer, sizeof(txBuffer),
             "pos1:%.2f vel1:%.2f pos2:%.2f vel2:%.2f pos3:%.2f vel3:%.2f pos4:%.2f vel4:%.2f yaw:%.2f",
             pos1, velO1, pos2, velO2, pos3, velO3, pos4, velO4, yaw);

    // Check if snprintf was successful
    if (length < 0 || length >= sizeof(txBuffer)) {
        // Handle error (e.g., log it, return, etc.)
        // For example, you could print an error message or set an error flag
        return; // Exit the function if there was an error
    }

    // Transmit the complete message over UART
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, (uint8_t*) txBuffer, strlen(txBuffer), HAL_MAX_DELAY);

    // Check if the transmission was successful
    if (status != HAL_OK) {
        // Handle the error (e.g., log it, return, etc.)
        // You can use HAL_UART_GetError(&huart2) to get more information about the error
        // For example:
        uint32_t error = HAL_UART_GetError(&huart2);
        // Log the error or take appropriate action
        printf("UART Transmission Error: %lu\n", error);
    }
}
    // Optional delay to prevent flooding the UART
*/



/// REVICE DATA ///

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
    // Example input: "c: 0.54,0.54,0.54,0.54"

    // Print the received buffer for debugging
    printf("Received buffer: %s\n", buffer);


    // Pointer to the start of the buffer
    char *start = (char*) buffer;

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

        float newValue1 = 0.0, newValue2 = 0.0, newValue3 = 0.0, newValue4 = 0.0;
        static float oldValue1 = 0.0, oldValue2 = 0.0, oldValue3 = 0.0, oldValue4 = 0.0;

        if (token != NULL) {
            newValue1 = atof(token); // Convert to float
            // printf("Parsed val1: %.2f\n", newValue1);
        }

        token = strtok(NULL, ",");
        if (token != NULL) {
            newValue2 = atof(token); // Convert to float
            printf("Parsed val2: %.2f\n", newValue2);
        }

        token = strtok(NULL, ",");
        if (token != NULL) {
            newValue3 = atof(token); // Convert to float
            // printf("Parsed val3: %.2f\n", newValue3);
        }

        token = strtok(NULL, ",");
        if (token != NULL) {
            newValue4 = atof(token); // Convert to float
            // printf("Parsed val4: %.2f\n", newValue4);
        }

        // Check if new values are different from old values
        if (newValue1 != oldValue1 || newValue2 != oldValue2
                || newValue3 != oldValue3 || newValue4 != oldValue4) {
            *val1 = newValue1;
            *val2 = newValue2;
            *val3 = newValue3;
            *val4 = newValue4;

            oldValue1 = newValue1;
            oldValue2 = newValue2;
            oldValue3 = newValue3;
            oldValue4 = newValue4;
        }

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
void resetDataSend(void) {
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
/*void SR(void) {

	ReadFourFloats(&value1, &value2, &value3, &value4);

	//-12.34,56.78,-90.12,34.56 Input example
	//Send back the received values (for verification)
	  char output[100];
	 snprintf(output, sizeof(output), "Received: %.2f, %.2f, %.2f, %.2f\r\n", value1, value2, value3, value4);
	 HAL_UART_Transmit(&huart2, (uint8_t *)output, strlen(output), HAL_MAX_DELAY);

	sendJointState(pos1, pos2, pos3, pos4, velR1, velR2, velR3, velR4);

}*/

uint32_t time;


#define MOVING_AVERAGE_SIZE 5 // Size for moving average filter
#define PPR 11 // Pulses per revolution
#define MAX_VELOCITY 0.27 // Maximum linear velocity in m/s
#define MAX_PWM_VALUE 1000 // Maximum PWM value (based on TIM12 period)
#define MAX_INTEGRAL 100.0 // Maximum integral value to prevent windup
#define SMOOTHING_FACTOR 0.2 // Adjusted factor for low-pass filter for quicker response
#define RATE_LIMIT 10.0 // Maximum change in control output per cycle
#define VELOCITY_CHANGE_THRESHOLD 0.54 // Threshold for sudden velocity change
#define VELOCITY_PROXIMITY_THRESHOLD 0.02 // Threshold for proximity to desired velocity
#define STOP_DURATION 100 // Duration to stop the motor in milliseconds
#define DEAD_BAND 0.01 // Deadband for velocity control
#define HYSTERESIS 0.03 // Hysteresis for switching between positive and negative
#define RAMP_RATE 5.0 // Maximum change in control output per cycle for ramping

// Global variables
uint32_t pulse_count1 = 0;  // Variable to store pulse count
double rpm1 = 0.0;           // Variable to store calculated RPM
float vel1 = 0.0;            // Linear velocity (m/s)
float dia1 = 0.097;          // Diameter in meters

float Kp1 = 0.1;             // Proportional gain
float Ki1 = 0.01;            // Integral gain
float Kd1 = 0.25;            // Derivative gain
float control_output1;       // Control output for PWM
float integral1_1 = 0.0;      // Integral term for PID
float last_error1 = 0.0;     // Last error for PID
float last_control_output1 = 0.0; // Last control output for smoothing
float last_valid_vel1 = 0.0; // Last valid velocity
float last_velTag1 = 0.0;    // Store the last velTag

// Kalman filter variables
float kalman_gain1 = 0.1; // Initial Kalman gain
float estimate1 = 0.0;     // Initial estimate of velocity
float error_covariance1 = 1.0; // Initial error covariance
float process_noise1 = 0.1; // Process noise covariance
float measurement_noise1 = 0.1; // Measurement noise covariance

// Moving average buffer
float velocity_buffer1[MOVING_AVERAGE_SIZE] = { 0 };
int buffer_index1 = 0;
float last_time1 = 0.0; // Variable to store the last time update
float angular_position_rad1 = 0.0; // Angular position in radians
float angular_position_deg1 = 0.0; // Angular position in degrees
float realVel1;
float realRPM1;

// Function to calculate exponential moving average
float moving_average_filter1(float new_velocity) {
	static float ema1 = 0.0; // Initialize EMA variable
	ema1 = (SMOOTHING_FACTOR * new_velocity) + ((1 - SMOOTHING_FACTOR) * ema1);
	return ema1;
}

// Function to calculate PWM duty cycle based on desired velocity
float calculate_pwm1(float desired_velocity) {
	if (desired_velocity < 0) {
		desired_velocity = -desired_velocity;
	}
	return (desired_velocity / MAX_VELOCITY) * MAX_PWM_VALUE; // Scale to PWM range
}

// PID Controller Function with Anti-Windup
float PID_Controller1(float Kp, float Ki, float Kd, float *integral,
		float last_error, float setpoint, float measured_value) {
	// Calculate the error
	float error1 = setpoint - measured_value;

	// Update the integral term with clamping to prevent windup
	*integral += error1;
	if (*integral > MAX_INTEGRAL) {
		*integral = MAX_INTEGRAL; // Clamp integral to prevent windup
	} else if (*integral < -MAX_INTEGRAL) {
		*integral = -MAX_INTEGRAL; // Clamp integral to prevent windup
	}

	// Calculate the derivative term
	float derivative1 = error1 - last_error;

	// Calculate the output
	float output1 = (Kp * error1) + (Ki * (*integral)) + (Kd * derivative1);

	// Save the last error for next iteration
	last_error = error1;

	return output1; // Return the control output
}

int32_t current_pulse_count1 = 0;
 float distance_traveled1 = 0.0;
// Function to calculate RPM and control the motor
void calculateVel1(float velTag1, float current_time1) {


	// Check if velTag1 is within the deadband
	if (fabs(velTag1) < DEAD_BAND) {
		velTag1 = 0; // Set velTag1 to zero if within deadband
	}

	// Immediate stop if velTag1 is 0
	if (velTag1 == 0) {
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
		vel1 = 0.0;
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		rpm1 = 0.0;
		control_output1 = 0.0;
		// Reset variables except angular_position_rad1
		realVel1 = 0.0;
		realRPM1 = 0.0;
		integral1_1 = 0.0;
		last_error1 = 0.0;
		last_control_output1 = 0.0;
		distance_traveled1 = 0.0;
		pulse_count1 = 0; // Reset pulse count
		last_velTag1 = velTag1; // Update last velTag1
		HAL_Delay(STOP_DURATION); // Wait for 100 ms
		return; // Exit the function
	}

	// Calculate the time elapsed since the last update
	float delta_time1 = current_time1 - last_time1;

	// Read the current pulse count
	current_pulse_count1 = __HAL_TIM_GET_COUNTER(&htim1);
	HAL_Delay(10);

	// Calculate the difference in pulse count
	int32_t pulse_difference1 = current_pulse_count1 - pulse_count1;

	// Calculate RPM as a positive value
	rpm1 = fabs((float) pulse_difference1 / (float) PPR) * 60.0; // Always positive
	pulse_count1 = current_pulse_count1;

	// Limit RPM to the range [0, 250]
	rpm1 = fmax(0.0, fmin(250.0, rpm1));
	// Calculate linear velocity (m/s)
	float new_vel1;
	if (pulse_difference1 < 0) {
		new_vel1 = -((rpm1 / 60.0) * dia1 * M_PI); // Negative velocity for reverse direction
	} else {
		new_vel1 = (rpm1 / 60.0) * dia1 * M_PI; // Positive velocity for forward direction
	}

	// Apply moving average filter for velocity
	//  vel1 = moving_average_filter1(new_vel1);
	// Constrain the velocity to the range [-1, 1]
	vel1 = fmax(-1.0, fmin(1.0, moving_average_filter1(new_vel1)));
	// Update position based on velocity and elapsed time
	distance_traveled1 += vel1 * (delta_time1 / 1000.0); // Linear distance traveled in meters
	angular_position_rad1 += distance_traveled1 / (dia1 / 2.0); // Update angular position in radians
	angular_position_deg1 = angular_position_rad1 * (180.0 / M_PI); // Convert to degrees

	// Kalman filter update
	estimate1 = estimate1; // Predicted state (previous estimate)
	error_covariance1 += process_noise1; // Update error covariance

	// Measurement update
	kalman_gain1 = error_covariance1 / (error_covariance1 + measurement_noise1); // Calculate Kalman gain
	estimate1 += kalman_gain1 * (vel1 - estimate1); // Update estimate with measurement
	error_covariance1 = (1 - kalman_gain1) * error_covariance1; // Update error covariance

	// Calculate control output using PID controller
	control_output1 = PID_Controller1(Kp1, Ki1, Kd1, &integral1_1, last_error1,
			velTag1, vel1);

	// Implement ramping to control output
	if (fabs(control_output1 - last_control_output1) > RAMP_RATE) {
		control_output1 = last_control_output1
				+ (control_output1 > last_control_output1 ?
						RAMP_RATE : -RAMP_RATE);
	}

	// Implement hysteresis to prevent rapid switching
	if ((last_control_output1 > 0 && control_output1 < -HYSTERESIS)
			|| (last_control_output1 < 0 && control_output1 > HYSTERESIS)) {
		control_output1 = last_control_output1; // Maintain last control output if within hysteresis
	}

	realVel1 = vel1 / 2.0; // Scale factor
	realRPM1 = rpm1 / 2.0;
	if(realVel1 <= 0.01 && velTag1 >= 0.0 ){
			realVel1 =0.0;
		}
		else if ( realVel1 >= -0.01 && velTag1 <= 0.0){
			realVel1 =0.0;
		}
	// Limit control_output4 to the range [-0.27, 0.27]
	control_output1 = fmax(-0.27, fmin(0.27, control_output1));
	// Set the PWM duty cycle based on the sign of desired_velocity
	if (velTag1 > 0) {
		// Positive velocity
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2,
				calculate_pwm1(control_output1));
	} else if (velTag1 < 0) {
		// Negative velocity
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1,
				calculate_pwm1(control_output1));
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
	}

	// Update last time and last control output
	last_time1 = current_time1;
	last_control_output1 = control_output1;
}
// Global variables
uint32_t pulse_count2 = 0;  // Variable to store pulse count
double rpm2 = 0.0;           // Variable to store calculated RPM
float vel2 = 0.0;            // Linear velocity (m/s)
float dia2 = 0.097;          // Diameter in meters

float Kp2 = 0.15;             // Proportional gain
float Ki2 = 0.01;            // Integral gain
float Kd2 = 0.25;            // Derivative gain
float control_output2;       // Control output for PWM
float integral1_2 = 0.0;      // Integral term for PID
float last_error2 = 0.0;     // Last error for PID
float last_control_output2 = 0.0; // Last control output for smoothing
float last_valid_vel2 = 0.0; // Last valid velocity
float last_velTag2 = 0.0;    // Store the last velTag

// Kalman filter variables
float kalman_gain2 = 0.1; // Initial Kalman gain
float estimate2 = 0.0;     // Initial estimate of velocity
float error_covariance2 = 1.0; // Initial error covariance
float process_noise2 = 0.1; // Process noise covariance
float measurement_noise2 = 0.1; // Measurement noise covariance

// Moving average buffer
float velocity_buffer2[MOVING_AVERAGE_SIZE] = { 0 };
int buffer_index2 = 0;
float last_time2 = 0.0; // Variable to store the last time update
float angular_position_rad2 = 0.0; // Angular position in radians
float angular_position_deg2 = 0.0; // Angular position in degrees
float realVel2;
float realRPM2;

// Function to calculate exponential moving average
float moving_average_filter2(float new_velocity) {
	static float ema2 = 0.0; // Initialize EMA variable
	ema2 = (SMOOTHING_FACTOR * new_velocity) + ((1 - SMOOTHING_FACTOR) * ema2);
	return ema2;
}

// Function to calculate PWM duty cycle based on desired velocity
float calculate_pwm2(float desired_velocity) {
	if (desired_velocity < 0) {
		desired_velocity = -desired_velocity;
	}
	return (desired_velocity / MAX_VELOCITY) * MAX_PWM_VALUE; // Scale to PWM range
}

// PID Controller Function with Anti-Windup
float PID_Controller2(float Kp, float Ki, float Kd, float *integral,
		float last_error, float setpoint, float measured_value) {
	// Calculate the error
	float error2 = setpoint - measured_value;

	// Update the integral term with clamping to prevent windup
	*integral += error2;
	if (*integral > MAX_INTEGRAL) {
		*integral = MAX_INTEGRAL; // Clamp integral to prevent windup
	} else if (*integral < -MAX_INTEGRAL) {
		*integral = -MAX_INTEGRAL; // Clamp integral to prevent windup
	}

	// Calculate the derivative term
	float derivative2 = error2 - last_error;

	// Calculate the output
	float output2 = (Kp * error2) + (Ki * (*integral)) + (Kd * derivative2);

	// Save the last error for next iteration
	last_error = error2;

	return output2; // Return the control output
}

int32_t current_pulse_count2 = 0;
float distance_traveled2 = 0.0;
// Function to calculate RPM and control the motor
void calculateVel2(float velTag2, float current_time2) {


	// Check if velTag2 is within the deadband
	if (fabs(velTag2) < DEAD_BAND) {
		velTag2 = 0; // Set velTag2 to zero if within deadband
	}

	// Immediate stop if velTag2 is 0
	if (velTag2 == 0) {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
		vel2 = 0.0;
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		rpm2 = 0.0;
		control_output2 = 0.0;
		// Reset variables except angular_position_rad2
		realVel2 = 0.0;
		realRPM2 = 0.0;
		integral1_2 = 0.0;
		last_error2 = 0.0;
		last_control_output2 = 0.0;
		distance_traveled2 = 0.0;
		pulse_count2 = 0; // Reset pulse count
		last_velTag2 = velTag2; // Update last velTag2
		HAL_Delay(STOP_DURATION); // Wait for 100 ms
		return; // Exit the function
	}

	// Calculate the time elapsed since the last update
	float delta_time2 = current_time2 - last_time2;

	// Read the current pulse count
	current_pulse_count2 = __HAL_TIM_GET_COUNTER(&htim3);
	HAL_Delay(10);

	// Calculate the difference in pulse count
	int32_t pulse_difference2 = current_pulse_count2 - pulse_count2;

	// Calculate RPM as a positive value
	rpm2 = fabs((float) pulse_difference2 / (float) PPR) * 60.0; // Always positive
	pulse_count2 = current_pulse_count2;
	// Limit RPM to the range [0, 250]
	rpm2 = fmax(0.0, fmin(250.0, rpm2));
	// Calculate linear velocity (m/s)
	float new_vel2;
	if (pulse_difference2 < 0) {
		new_vel2 = -((rpm2 / 60.0) * dia2 * M_PI); // Negative velocity for reverse direction
	} else {
		new_vel2 = (rpm2 / 60.0) * dia2 * M_PI; // Positive velocity for forward direction
	}

	// Apply moving average filter for velocity
	//vel2 = moving_average_filter2(new_vel2);
	// Constrain the velocity to the range [-1, 1]
	vel2 = fmax(-1.0, fmin(1.0, moving_average_filter2(new_vel2)));
	// Update position based on velocity and elapsed time
	distance_traveled2 += vel2 * (delta_time2 / 1000.0); // Linear distance traveled in meters
	angular_position_rad2 += distance_traveled2 / (dia2 / 2.0); // Update angular position in radians
	angular_position_deg2 = angular_position_rad2 * (180.0 / M_PI); // Convert to degrees

	// Kalman filter update
	estimate2 = estimate2; // Predicted state (previous estimate)
	error_covariance2 += process_noise2; // Update error covariance

	// Measurement update
	kalman_gain2 = error_covariance2 / (error_covariance2 + measurement_noise2); // Calculate Kalman gain
	estimate2 += kalman_gain2 * (vel2 - estimate2); // Update estimate with measurement
	error_covariance2 = (1 - kalman_gain2) * error_covariance2; // Update error covariance

	// Calculate control output using PID controller
	control_output2 = PID_Controller2(Kp2, Ki2, Kd2, &integral1_2, last_error2,
			velTag2, vel2);

	// Implement ramping to control output
	if (fabs(control_output2 - last_control_output2) > RAMP_RATE) {
		control_output2 = last_control_output2
				+ (control_output2 > last_control_output2 ?
						RAMP_RATE : -RAMP_RATE);
	}

	// Implement hysteresis to prevent rapid switching
	if ((last_control_output2 > 0 && control_output2 < -HYSTERESIS)
			|| (last_control_output2 < 0 && control_output2 > HYSTERESIS)) {
		control_output2 = last_control_output2; // Maintain last control output if within hysteresis
	}

	realVel2 = vel2 / 2.0; // Scale factor
	realRPM2 = rpm2 / 2.0;
	if(realVel2 <= 0.01 && velTag2 >= 0.0 ){
		realVel2 =0.0;
	}
	else if ( realVel2 >= -0.01 && velTag2 <= 0.0){
		realVel2 =0.0;
	}
	// Limit control_output4 to the range [-0.27, 0.27]
	control_output2 = fmax(-0.27, fmin(0.27, control_output2));
	// Set the PWM duty cycle based on the sign of desired_velocity
	if (velTag2 > 0) {
		// Positive velocity
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,
				calculate_pwm2(control_output2));
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
	} else if (velTag2 < 0) {
		// Negative velocity
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,
				calculate_pwm2(control_output2));

	}

	// Update last time and last control output
	last_time2 = current_time2;
	last_control_output2 = control_output2;
}
// Global variables
uint32_t pulse_count3 = 0;  // Variable to store pulse count
double rpm3 = 0.0;           // Variable to store calculated RPM
float vel3 = 0.0;            // Linear velocity (m/s)
float dia3 = 0.097;          // Diameter in meters

float Kp3 = 0.15;             // Proportional gain
float Ki3 = 0.01;            // Integral gain
float Kd3 = 0.25;            // Derivative gain
float control_output3;       // Control output for PWM
float integral1_3 = 0.0;      // Integral term for PID
float last_error3 = 0.0;     // Last error for PID
float last_control_output3 = 0.0; // Last control output for smoothing
float last_valid_vel3 = 0.0; // Last valid velocity
float last_velTag3 = 0.0;    // Store the last velTag

// Kalman filter variables
float kalman_gain3 = 0.1; // Initial Kalman gain
float estimate3 = 0.0;     // Initial estimate of velocity
float error_covariance3 = 1.0; // Initial error covariance
float process_noise3 = 0.1; // Process noise covariance
float measurement_noise3 = 0.1; // Measurement noise covariance

// Moving average buffer
float velocity_buffer3[MOVING_AVERAGE_SIZE] = { 0 };
int buffer_index3 = 0;
float last_time3 = 0.0; // Variable to store the last time update
float angular_position_rad3 = 0.0; // Angular position in radians
float angular_position_deg3 = 0.0; // Angular position in degrees
float realVel3;
float realRPM3;

// Function to calculate exponential moving average
float moving_average_filter3(float new_velocity) {
	static float ema3 = 0.0; // Initialize EMA variable
	ema3 = (SMOOTHING_FACTOR * new_velocity) + ((1 - SMOOTHING_FACTOR) * ema3);
	return ema3;
}

// Function to calculate PWM duty cycle based on desired velocity
float calculate_pwm3(float desired_velocity) {
	if (desired_velocity < 0) {
		desired_velocity = -desired_velocity;
	}
	return (desired_velocity / MAX_VELOCITY) * MAX_PWM_VALUE; // Scale to PWM range
}

// PID Controller Function with Anti-Windup
float PID_Controller3(float Kp, float Ki, float Kd, float *integral,
		float last_error, float setpoint, float measured_value) {
	// Calculate the error
	float error3 = setpoint - measured_value;

	// Update the integral term with clamping to prevent windup
	*integral += error3;
	if (*integral > MAX_INTEGRAL) {
		*integral = MAX_INTEGRAL; // Clamp integral to prevent windup
	} else if (*integral < -MAX_INTEGRAL) {
		*integral = -MAX_INTEGRAL; // Clamp integral to prevent windup
	}

	// Calculate the derivative term
	float derivative3 = error3 - last_error;

	// Calculate the output
	float output3 = (Kp * error3) + (Ki * (*integral)) + (Kd * derivative3);

	// Save the last error for next iteration
	last_error = error3;

	return output3; // Return the control output
}

int32_t current_pulse_count3 = 0;
float distance_traveled3 = 0.0;
// Function to calculate RPM and control the motor
void calculateVel3(float velTag3, float current_time3) {


	// Check if velTag3 is within the deadband
	if (fabs(velTag3) < DEAD_BAND) {
		velTag3 = 0; // Set velTag3 to zero if within deadband
	}

	// Immediate stop if velTag3 is 0
	if (velTag3 == 0) {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
		vel3 = 0.0;
		__HAL_TIM_SET_COUNTER(&htim5, 0);
		rpm3 = 0.0;
		control_output3 = 0.0;
		// Reset variables except angular_position_rad3
		realVel3 = 0.0;
		realRPM3 = 0.0;
		integral1_3 = 0.0;
		last_error3 = 0.0;
		last_control_output3 = 0.0;
		distance_traveled3 = 0.0;
		pulse_count3 = 0; // Reset pulse count
		last_velTag3 = velTag3; // Update last velTag3
		HAL_Delay(STOP_DURATION); // Wait for 100 ms
		return; // Exit the function
	}

	// Calculate the time elapsed since the last update
	float delta_time3 = current_time3 - last_time3;

	// Read the current pulse count
	current_pulse_count3 = __HAL_TIM_GET_COUNTER(&htim5);
	HAL_Delay(10);

	// Calculate the difference in pulse count
	int32_t pulse_difference3 = current_pulse_count3 - pulse_count3;

	// Calculate RPM as a positive value
	rpm3 = fabs((float) pulse_difference3 / (float) PPR) * 60.0; // Always positive
	pulse_count3 = current_pulse_count3;
	// Limit RPM to the range [0, 250]
	rpm3 = fmax(0.0, fmin(250.0, rpm3));
	// Calculate linear velocity (m/s)
	float new_vel3;
	if (pulse_difference3 < 0) {
		new_vel3 = -((rpm3 / 60.0) * dia3 * M_PI); // Negative velocity for reverse direction
	} else {
		new_vel3 = (rpm3 / 60.0) * dia3 * M_PI; // Positive velocity for forward direction
	}
	// Constrain the velocity to the range [-1, 1]
	vel3 = fmax(-1.0, fmin(1.0, moving_average_filter3(new_vel3)));
	// Apply moving average filter for velocity
	// vel3 = moving_average_filter3(new_vel3);

	// Update position based on velocity and elapsed time
	distance_traveled3 += vel3 * (delta_time3 / 1000.0); // Linear distance traveled in meters
	angular_position_rad3 += distance_traveled3 / (dia3 / 2.0); // Update angular position in radians
	angular_position_deg3 = angular_position_rad3 * (180.0 / M_PI); // Convert to degrees

	// Kalman filter update
	estimate3 = estimate3; // Predicted state (previous estimate)
	error_covariance3 += process_noise3; // Update error covariance

	// Measurement update
	kalman_gain3 = error_covariance3 / (error_covariance3 + measurement_noise3); // Calculate Kalman gain
	estimate3 += kalman_gain3 * (vel3 - estimate3); // Update estimate with measurement
	error_covariance3 = (1 - kalman_gain3) * error_covariance3; // Update error covariance

	// Calculate control output using PID controller
	control_output3 = PID_Controller3(Kp3, Ki3, Kd3, &integral1_3, last_error3,
			velTag3, vel3);

	// Implement ramping to control output
	if (fabs(control_output3 - last_control_output3) > RAMP_RATE) {
		control_output3 = last_control_output3
				+ (control_output3 > last_control_output3 ?
						RAMP_RATE : -RAMP_RATE);
	}

	// Implement hysteresis to prevent rapid switching
	if ((last_control_output3 > 0 && control_output3 < -HYSTERESIS)
			|| (last_control_output3 < 0 && control_output3 > HYSTERESIS)) {
		control_output3 = last_control_output3; // Maintain last control output if within hysteresis
	}

	realVel3 = vel3 / 2.0; // Scale factor
	realRPM3 = rpm3 / 2.0;
	if(realVel3 <= 0.01 && velTag3 >= 0.0 ){
			realVel3 =0.0;
		}
		else if ( realVel3 >= -0.01 && velTag3 <= 0.0){
			realVel3 =0.0;
		}
	// Limit control_output4 to the range [-0.27, 0.27]
	control_output3 = fmax(-0.27, fmin(0.27, control_output3));
	// Set the PWM duty cycle based on the sign of desired_velocity
	if (velTag3 > 0) {
		// Positive velocity
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4,
				calculate_pwm3(control_output3));
	} else if (velTag3 < 0) {
		// Negative velocity
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,
				calculate_pwm3(control_output3));
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	}

	// Update last time and last control output
	last_time3 = current_time3;
	last_control_output3 = control_output3;
}

// Global variables
uint32_t pulse_count4 = 0;  // Variable to store pulse count
double rpm4 = 0.0;           // Variable to store calculated RPM
float vel4 = 0.0;            // Linear velocity (m/s)
float dia4 = 0.097;          // Diameter in meters

float Kp4 = 0.15;             // Proportional gain
float Ki4 = 0.01;            // Integral gain
float Kd4 = 0.25;            // Derivative gain
float control_output4;       // Control output for PWM
float integral1_4 = 0.0;      // Integral term for PID
float last_error4 = 0.0;     // Last error for PID
float last_control_output4 = 0.0; // Last control output for smoothing
float last_valid_vel4 = 0.0; // Last valid velocity
float last_velTag4 = 0.0;    // Store the last velTag

// Kalman filter variables
float kalman_gain4 = 0.1; // Initial Kalman gain
float estimate4 = 0.0;     // Initial estimate of velocity
float error_covariance4 = 1.0; // Initial error covariance
float process_noise4 = 0.1; // Process noise covariance
float measurement_noise4 = 0.1; // Measurement noise covariance

// Moving average buffer
float velocity_buffer4[MOVING_AVERAGE_SIZE] = { 0 };
int buffer_index4 = 0;
float last_time4 = 0.0; // Variable to store the last time update
float angular_position_rad4 = 0.0; // Angular position in radians
float angular_position_deg4 = 0.0; // Angular position in degrees
float realVel4;
float realRPM4;

// Function to calculate exponential moving average
float moving_average_filter4(float new_velocity) {
	static float ema4 = 0.0; // Initialize EMA variable
	ema4 = (SMOOTHING_FACTOR * new_velocity) + ((1 - SMOOTHING_FACTOR) * ema4);
	return ema4;
}

// Function to calculate PWM duty cycle based on desired velocity
float calculate_pwm4(float desired_velocity) {
	if (desired_velocity < 0) {
		desired_velocity = -desired_velocity;
	}
	return (desired_velocity / MAX_VELOCITY) * MAX_PWM_VALUE; // Scale to PWM range
}

// PID Controller Function with Anti-Windup
float PID_Controller4(float Kp, float Ki, float Kd, float *integral,
		float last_error, float setpoint, float measured_value) {
	// Calculate the error
	float error4 = setpoint - measured_value;

	// Update the integral term with clamping to prevent windup
	*integral += error4;
	if (*integral > MAX_INTEGRAL) {
		*integral = MAX_INTEGRAL; // Clamp integral to prevent windup
	} else if (*integral < -MAX_INTEGRAL) {
		*integral = -MAX_INTEGRAL; // Clamp integral to prevent windup
	}

	// Calculate the derivative term
	float derivative4 = error4 - last_error;

	// Calculate the output
	float output4 = (Kp * error4) + (Ki * (*integral)) + (Kd * derivative4);

	// Save the last error for next iteration
	last_error = error4;

	return output4; // Return the control output
}

int32_t current_pulse_count4 = 0;
float distance_traveled4 = 0.0;
// Function to calculate RPM and control the motor
void calculateVel4(float velTag4, float current_time4) {


	// Check if velTag4 is within the deadband
	if (fabs(velTag4) < DEAD_BAND) {
		velTag4 = 0; // Set velTag4 to zero if within deadband
	}

	// Immediate stop if velTag4 is 0
	if (velTag4 == 0) {
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
		vel4 = 0.0;
		__HAL_TIM_SET_COUNTER(&htim8, 0);
		rpm4 = 0.0;
		control_output4 = 0.0;
		// Reset variables except angular_position_rad4
		realVel4 = 0.0;
		realRPM4 = 0.0;
		integral1_4 = 0.0;
		last_error4 = 0.0;
		last_control_output4 = 0.0;
		distance_traveled4 = 0.0;
		pulse_count4 = 0; // Reset pulse count
		last_velTag4 = velTag4; // Update last velTag4
		HAL_Delay(STOP_DURATION); // Wait for 100 ms
		return; // Exit the function
	}

	// Calculate the time elapsed since the last update
	float delta_time4 = current_time4 - last_time4;

	// Read the current pulse count
	current_pulse_count4 = __HAL_TIM_GET_COUNTER(&htim8);
	HAL_Delay(10);

	// Calculate the difference in pulse count
	int32_t pulse_difference4 = current_pulse_count4 - pulse_count4;

	// Calculate RPM as a positive value
	rpm4 = fabs((float) pulse_difference4 / (float) PPR) * 60.0; // Always positive
	pulse_count4 = current_pulse_count4;
	// Limit RPM to the range [0, 250]
	rpm4 = fmax(0.0, fmin(250.0, rpm4));
	// Calculate linear velocity (m/s)
	float new_vel4;
	if (pulse_difference4 < 0) {
		new_vel4 = -((rpm4 / 60.0) * dia4 * M_PI); // Negative velocity for reverse direction
	} else {
		new_vel4 = (rpm4 / 60.0) * dia4 * M_PI; // Positive velocity for forward direction
	}

	// Constrain the velocity to the range [-1, 1]
	vel4 = fmax(-1.0, fmin(1.0, moving_average_filter4(new_vel4)));
	// Apply moving average filter for velocity
	//  vel4 = moving_average_filter4(new_vel4);

	// Update position based on velocity and elapsed time
	distance_traveled4 += vel4 * (delta_time4 / 1000.0); // Linear distance traveled in meters
	angular_position_rad4 += distance_traveled4 / (dia4 / 2.0); // Update angular position in radians
	angular_position_deg4 = angular_position_rad4 * (180.0 / M_PI); // Convert to degrees

	// Kalman filter update
	estimate4 = estimate4; // Predicted state (previous estimate)
	error_covariance4 += process_noise4; // Update error covariance

	// Measurement update
	kalman_gain4 = error_covariance4 / (error_covariance4 + measurement_noise4); // Calculate Kalman gain
	estimate4 += kalman_gain4 * (vel4 - estimate4); // Update estimate with measurement
	error_covariance4 = (1 - kalman_gain4) * error_covariance4; // Update error covariance
	// Calculate control output using PID controller
	control_output4 = PID_Controller4(Kp4, Ki4, Kd4, &integral1_4, last_error4,
			velTag4, vel4);

	// Implement ramping to control output
	if (fabs(control_output4 - last_control_output4) > RAMP_RATE) {
		control_output4 = last_control_output4
				+ (control_output4 > last_control_output4 ?
						RAMP_RATE : -RAMP_RATE);
	}

	// Implement hysteresis to prevent rapid switching
	if ((last_control_output4 > 0 && control_output4 < -HYSTERESIS)
			|| (last_control_output4 < 0 && control_output4 > HYSTERESIS)) {
		control_output4 = last_control_output4; // Maintain last control output if within hysteresis
	}

	realVel4 = vel4 / 2.0; // Scale factor
	realRPM4 = rpm4 / 2.0;
	if(realVel4 <= 0.01 && velTag4 >= 0.0 ){
			realVel4 =0.0;
		}
		else if ( realVel4 >= -0.01 && velTag4 <= 0.0){
			realVel4 =0.0;
		}
	// Limit control_output4 to the range [-0.27, 0.27]
	control_output4 = fmax(-0.27, fmin(0.27, control_output4));

	// Set the PWM duty cycle based on the sign of desired_velocity
	if (velTag4 > 0) {
		// Positive velocity
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1,
				calculate_pwm4(control_output4));
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);

	} else if (velTag4 < 0) {
		// Negative velocity
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2,
				calculate_pwm4(control_output4));
	}

	// Update last time and last control output
	last_time4 = current_time4;
	last_control_output4 = control_output4;
}

void motor(void) {
//BNO055_Read_Euler_Quaternion(&hi2c1, &yaw, &pitch, &roll, &qx, &qy, &qz, &qw);
	ReadFourFloats(&value1, &value2, &value3, &value4);
	readBNO055();
	HAL_Delay(1);

	time = get_custom_tick();
	calculateVel1(value1, time);
	calculateVel2(value2, time);
	calculateVel3(value3, time);
    calculateVel4(value4, time);
  /*  calculateVel1(0.54, time);
	calculateVel2(0.54, time);
	calculateVel3(0.54, time);
	calculateVel4(0.54, time);*/

	// Print the final values
	HAL_Delay(100);
	sendJointState(angular_position_rad1, angular_position_rad2,
			angular_position_rad3, angular_position_rad4,
			realVel1, realVel2,realVel3, realVel4,yaw);

}
