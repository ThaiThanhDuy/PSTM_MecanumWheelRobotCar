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
    // c: 0.15,0.15,0.15,0.15
    // Print the received buffer for debugging
 //   printf("Received buffer: %s\n", buffer);

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
    //    printf("Data after prefix: %s\n", data);

        // Parse the string
        char *token = strtok(data, ",");
        if (token != NULL) {
            *val1 = atof(token); // Convert to float and store
 //           printf("Parsed val1: %.2f\n", *val1);
        }

        token = strtok(NULL, ",");
        if (token != NULL) {
            *val2 = atof(token); // Convert to float and store
     //       printf("Parsed val2: %.2f\n", *val2);
        }

        token = strtok(NULL, ",");
        if (token != NULL) {
            *val3 = atof(token); // Convert to float and store
    //        printf("Parsed val3: %.2f\n", *val3);
        }

        token = strtok(NULL, ",");
        if (token != NULL) {
            *val4 = atof(token); // Convert to float and store
    //        printf("Parsed val4: %.2f\n", *val4);
        }

        // Sum the values
 //       float sum = *val1 + *val2 + *val3 + *val4;
   //     printf("Sum of values: %.2f\n", sum);

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

uint32_t time;

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


/*
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
         if(currentTime>=1215){
            Motor_Control1(0, 0); // time error
        }
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
}*/


#include <stdio.h>
#include <math.h>
#include <stdint.h>

#define MOVING_AVERAGE_SIZE 5 // Size for moving average filter
#define PPR 11 // Pulses per revolution
#define MAX_VELOCITY 0.27 // Maximum linear velocity in m/s
#define MAX_PWM_VALUE 1000 // Maximum PWM value (based on TIM12 period)
#define MAX_INTEGRAL 100.0 // Maximum integral value to prevent windup
#define SMOOTHING_FACTOR 0.2 // Adjusted factor for low-pass filter for quicker response
#define RATE_LIMIT 10.0 // Maximum change in control output per cycle
#define VELOCITY_CHANGE_THRESHOLD 0.1 // Threshold for sudden velocity change
#define VELOCITY_PROXIMITY_THRESHOLD 0.02 // Threshold for proximity to desired velocity

// Global variables
uint32_t pulse_count = 0;  // Variable to store pulse count
double rpm = 0.0;           // Variable to store calculated RPM
float vel = 0.0;           // Linear velocity (m/s)
float dia = 0.097;         // Diameter in meters

float Kp = 0.11;            // Proportional gain
float Ki = 0.0065;          // Integral gain
float Kd = 0.25;            // Derivative gain
float control_output;      // Control output for PWM
float integral1 = 0.0;     // Integral term for PID
float last_error = 0.0;    // Last error for PID
float last_control_output = 0.0; // Last control output for smoothing
float last_valid_vel = 0.0; // Last valid velocity

// Kalman filter variables
float kalman_gain = 0.1; // Initial Kalman gain
float estimate = 0.0;     // Initial estimate of velocity
float error_covariance = 1.0; // Initial error covariance
float process_noise = 0.1; // Process noise covariance
float measurement_noise = 0.1; // Measurement noise covariance

// Moving average buffer
float velocity_buffer[MOVING_AVERAGE_SIZE] = {0};
int buffer_index = 0;
 float last_time = 0.0; // Variable to store the last time update
 float angular_position_rad = 0.0; // Angular position in radians
 float angular_position_deg = 0.0; // Angular position in degrees
 float realVel ;
  float realRPM ;
// Function to calculate moving average
float moving_average_filter(float new_velocity) {
    velocity_buffer[buffer_index] = new_velocity;
    buffer_index = (buffer_index + 1) % MOVING_AVERAGE_SIZE;

    float sum = 0.0;
    for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
        sum += velocity_buffer[i];
    }
    return sum / MOVING_AVERAGE_SIZE;
}

// Function to calculate PWM duty cycle based on desired velocity
float calculate_pwm(float desired_velocity) {
    if (desired_velocity > MAX_VELOCITY) {
        desired_velocity = MAX_VELOCITY; // Limit to max velocity
    }
    if (desired_velocity < -MAX_VELOCITY) {
        desired_velocity = -MAX_VELOCITY; // Limit to min velocity
    }
    return (desired_velocity / MAX_VELOCITY) * MAX_PWM_VALUE; // Scale to PWM range
}

// PID Controller Function with Anti-Windup
float PID_Controller(float Kp, float Ki, float Kd, float *integral, float last_error, float setpoint, float measured_value) {
    // Calculate the error
    float error = setpoint - measured_value;

    // Update the integral term with clamping to prevent windup
    *integral += error;
    if (*integral > MAX_INTEGRAL) {
        *integral = MAX_INTEGRAL; // Clamp integral to prevent windup
    } else if (*integral < -MAX_INTEGRAL) {
        *integral = -MAX_INTEGRAL; // Clamp integral to prevent windup
    }

    // Calculate the derivative term
    float derivative = error - last_error;

    // Calculate the output
    float output = (Kp * error) + (Ki * (*integral)) + (Kd * derivative);

    return output; // Return the control output
}

// Function to calculate RPM and control the motor
void calculateVel1(float velTag, float current_time) {
    // Debugging output



    static float distance_traveled = 0.0;
    static int32_t current_pulse_count = 0;

    // Calculate the time elapsed since the last update
    float delta_time = current_time - last_time;

    // Read the current pulse count
    current_pulse_count = Read_Encoder1();

    HAL_Delay(100);

    // Calculate the difference in pulse count
    int32_t pulse_difference = current_pulse_count - pulse_count;


    // Calculate RPM as a positive value
    rpm = fabs((float)pulse_difference / (float)PPR) * 60.0; // Always positive
    pulse_count = current_pulse_count;

    // Calculate linear velocity (m/s)
    float new_vel;
    if (pulse_difference < 0) {
        new_vel = -((rpm / 60.0) * dia * M_PI); // Negative velocity for reverse direction
    } else {
        new_vel = (rpm / 60.0) * dia * M_PI; // Positive velocity for forward direction
    }

    // Apply moving average filter for velocity
    vel = moving_average_filter(new_vel);

    // Update position based on velocity and elapsed time
    distance_traveled = vel * (delta_time / 1000.0); // Linear distance traveled in meters
    angular_position_rad += distance_traveled / (dia / 2.0); // Update angular position in radians
    angular_position_deg = angular_position_rad * (180.0 / M_PI); // Convert to degrees

    // Kalman filter update
    estimate = estimate; // Predicted state (previous estimate)
    error_covariance += process_noise; // Update error covariance

    // Measurement update
    kalman_gain = error_covariance / (error_covariance + measurement_noise); // Calculate Kalman gain
    estimate += kalman_gain * (vel - estimate); // Update estimate with measurement
    error_covariance *= (1 - kalman_gain); // Update error covariance

    // Use the Kalman filter estimate for velocity
    float filtered_vel = estimate;

    // Check for sudden changes in velocity with deadband
    if (fabs(filtered_vel - last_valid_vel) > VELOCITY_CHANGE_THRESHOLD) {
        vel = last_valid_vel; // Ignore sudden change, retain last valid velocity
    } else {
        vel = filtered_vel; // Update velocity if change is within threshold
        last_valid_vel = vel; // Update last valid velocity
    }

    // Calculate control output using PID controller
    control_output = PID_Controller(Kp, Ki, Kd, &integral1, last_error, velTag, vel);


    // Maintain control output if within proximity threshold
    if (fabs(vel - velTag) < VELOCITY_PROXIMITY_THRESHOLD) {
        control_output = last_control_output; // Retain last control output
    }

    // Apply low-pass filter to smooth control output
    control_output = (SMOOTHING_FACTOR * control_output) + ((1 - SMOOTHING_FACTOR) * last_control_output);

    // Rate limit the control output to prevent sudden changes
    float output_change = control_output - last_control_output;
    if (output_change > RATE_LIMIT) {
        control_output = last_control_output + RATE_LIMIT; // Limit increase
    } else if (output_change < -RATE_LIMIT) {
        control_output = last_control_output - RATE_LIMIT; // Limit decrease
    }

    last_control_output = control_output; // Update last control output
    realVel = vel / 2.0; // Scale factor
    realRPM = rpm / 2.0;

    // Set the PWM duty cycle based on the sign of desired_velocity
    if (velTag > 0) {
        Motor_Control1(2, calculate_pwm(control_output)); // Positive velocity
    } else if (velTag < 0) {
        Motor_Control1(1, calculate_pwm(control_output)); // Negative velocity
    }

    // Update the last time for the next calculation
    last_time = current_time; // Store the current time for the next update
}
void motor(void){

	 // ReadFourFloats(&value1, &value2, &value3, &value4);
	  HAL_Delay(100);
	  time = HAL_GetTick();
	  HAL_Delay(1);
	 // JointState1 jointState1 = calculateVel1(0., time);
	  //JointState2 jointState2 = read2(&value2,currentTime);
	  //JointState3 jointState3 = read3(&value3,currentTime);
	  //JointState4 jointState4 = read4(&value4,currentTime);
	//  calculateVel1(0.54, time);
	  HAL_Delay(1);
	// sendJointState(angular_position_rad, 0.0, 0.0, 0.0,
			// realVel,  0.27,  0.27,  0.27);
	/*  sendJointState(jointState1.position,jointState2.position,  jointState3.position, jointState4.position,
 jointState1.angularVelocity, jointState2.angularVelocity,   jointState3.angularVelocity, jointState4.angularVelocity);
	  HAL_Delay(10);*/
}
