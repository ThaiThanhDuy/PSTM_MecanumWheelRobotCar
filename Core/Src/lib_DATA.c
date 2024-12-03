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
uint8_t buffer[30]; // Buffer to hold the received string
float value1, value2, value3, value4;
float oldValue1, oldValue2 ,oldValue3,oldValue4;
/// SEND DATA

void sendJointState(float pos1, float pos2, float pos3, float pos4, float vel1, float vel2, float vel3, float vel4) {
    // Prepare joint state message
    /*sprintf(txBuffer, "pos1:%i vel1:%i pos2:%i vel2:%i pos3:%i vel3:%i pos4:%i vel4:%i\n",
            pos1, vel1, pos2, vel2, pos3, vel3, pos4, vel4);*/

   /* snprintf(P1, sizeof(P1), "pos1:%.2f ", pos1);
    HAL_UART_Transmit(&huart2, (uint8_t*) P1, strlen(P1), HAL_MAX_DELAY);
    snprintf(V1, sizeof(V1), "vel1:%.2f ", vel1);
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

    HAL_Delay(100)*/;
    // Prepare a buffer to hold the complete joint state message
       char txBuffer[256]; // Ensure this buffer is large enough to hold the entire message

       // Format the joint state message into the buffer
       snprintf(txBuffer, sizeof(txBuffer), "pos1:%.2f vel1:%.2f pos2:%.2f vel2:%.2f pos3:%.2f vel3:%.2f pos4:%.2f vel4:%.2f\n",
                pos1, vel1, pos2, vel2, pos3, vel3, pos4, vel4);

       // Transmit the complete message over UART
       HAL_UART_Transmit(&huart2, (uint8_t*)txBuffer, strlen(txBuffer), HAL_MAX_DELAY);

       // Optional delay to prevent flooding the UART
       HAL_Delay(100);
}

/// REVICE DATA


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
	// Example input: "c: 0.15,0.15,0.15,0.15"

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
	    float newValue1, newValue2, newValue3, newValue4; // Store new values

	    if (token != NULL) {
	        newValue1 = atof(token); // Convert to float
	    //    printf("Parsed val1: %.2f\n", newValue1);
	    }

	    token = strtok(NULL, ",");
	    if (token != NULL) {
	        newValue2 = atof(token); // Convert to float
	        printf("Parsed val2: %.2f\n", newValue2);
	    }

	    token = strtok(NULL, ",");
	    if (token != NULL) {
	        newValue3 = atof(token); // Convert to float
	    //    printf("Parsed val3: %.2f\n", newValue3);
	    }

	    token = strtok(NULL, ",");
	    if (token != NULL) {
	        newValue4 = atof(token); // Convert to float
	       // printf("Parsed val4: %.2f\n", newValue4);
	    }

	    // Check if new values are different from old values
	    if (newValue1 != oldValue1 || newValue2 != oldValue2 || newValue3 != oldValue3 || newValue4 != oldValue4) {
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

// Global variables
uint32_t pulse_count = 0;  // Variable to store pulse count
double rpm = 0.0;           // Variable to store calculated RPM
float vel = 0.0;            // Linear velocity (m/s)
float dia = 0.097;          // Diameter in meters

float Kp = 0.15;             // Proportional gain
float Ki = 0.01;            // Integral gain
float Kd = 0.25;            // Derivative gain
float control_output;       // Control output for PWM
float integral1 = 0.0;      // Integral term for PID
float last_error = 0.0;     // Last error for PID
float last_control_output = 0.0; // Last control output for smoothing
float last_valid_vel = 0.0; // Last valid velocity
float last_velTag = 0.0;    // Store the last velTag

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
float realVel;
float realRPM;

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
    if (desired_velocity < 0) {
        desired_velocity = -desired_velocity;
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

    // Save the last error for next iteration
    last_error = error;

    return output; // Return the control output
}
int32_t current_pulse_count = 0;
// Function to calculate RPM and control the motor
void calculateVel1(float velTag, float current_time) {
    static float distance_traveled = 0.0;


    // Check if velTag has changed
    if (fabs(velTag - last_velTag) > VELOCITY_CHANGE_THRESHOLD) {
        // Stop the motor
    	Motor_Control1(0, 0);

        vel = 0.0;
        Reset_Encoder1();
        rpm = 0.0;
        // Reset variables
        integral1 = 0.0;
        last_error = 0.0;
        last_control_output = 0.0;
        distance_traveled = 0.0;
        pulse_count = 0; // Reset pulse count
        last_velTag = velTag; // Update last velTag
        HAL_Delay(STOP_DURATION); // Wait for 100 ms
    }

    // Calculate the time elapsed since the last update
    float delta_time = current_time - last_time;

    // Read the current pulse count
    current_pulse_count = Read_Encoder1();
    HAL_Delay(10);

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
    if(velTag == 0 ||velTag == -0 ){
    	  distance_traveled += 0.0 * (delta_time / 1000.0); // Linear distance traveled in meters
    	    angular_position_rad += distance_traveled / (dia / 2.0); // Update angular position in radians
    	    angular_position_deg = angular_position_rad * (180.0 / M_PI); // Convert to degrees
    }
    else {
    	  distance_traveled += vel * (delta_time / 1000.0); // Linear distance traveled in meters
    	    angular_position_rad += distance_traveled / (dia / 2.0); // Update angular position in radians
    	    angular_position_deg = angular_position_rad * (180.0 / M_PI); // Convert to degrees
    }


    // Kalman filter update
    estimate = estimate; // Predicted state (previous estimate)
    error_covariance += process_noise; // Update error covariance

    // Measurement update
    kalman_gain = error_covariance / (error_covariance + measurement_noise); // Calculate Kalman gain
    estimate += kalman_gain * (vel - estimate); // Update estimate with measurement
    error_covariance = (1 - kalman_gain) * error_covariance; // Update error covariance

    // Calculate control output using PID controller
    control_output = PID_Controller(Kp, Ki, Kd, &integral1, last_error, velTag, vel);

    // Apply rate limiting to control output
    if (fabs(control_output - last_control_output) > RATE_LIMIT) {
        control_output = last_control_output + (control_output > last_control_output ? RATE_LIMIT : -RATE_LIMIT);
    }

    realVel = vel / 2.0; // Scale factor
      realRPM = rpm / 2.0;

      // Set the PWM duty cycle based on the sign of desired_velocity
      if (velTag > 0) {
          Motor_Control1(2, calculate_pwm(control_output)); // Positive velocity
      } else if (velTag < 0) {
          Motor_Control1(1, calculate_pwm(control_output)); // Negative velocity
      }
    // Update last time and last control output
    last_time = current_time;
    last_control_output = control_output;


}
void motor(void){
		ReadFourFloats(&value1, &value2, &value3, &value4);


	  HAL_Delay(1);
	 // JointState1 jointState1 = calculateVel1(0., time);
	  //JointState2 jointState2 = read2(&value2,currentTime);
	  //JointState3 jointState3 = read3(&value3,currentTime);
	  //JointState4 jointState4 = read4(&value4,currentTime);
	  time = HAL_GetTick();
	 calculateVel1(value1, time);


	    // Print the final values
	  HAL_Delay(100);
	 sendJointState(angular_position_rad, 0.0, 0.0, 0.0,
			 realVel,  0.0,  0.0,  0.0);
	/*  sendJointState(jointState1.position,jointState2.position,  jointState3.position, jointState4.position,
 jointState1.angularVelocity, jointState2.angularVelocity,   jointState3.angularVelocity, jointState4.angularVelocity);
	  HAL_Delay(10);*/
}
