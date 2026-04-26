/*
 * pid_control.h
 *
 *  Created on: 07-Apr-2026
 *      Author: kasiviswanadhsripada
 */

#ifndef INC_PID_CONTROL_H_
#define INC_PID_CONTROL_H_


typedef struct {
    // Gains
    float kp;
    float ki;
    float kd;

    // State variables
    float integral;
    float last_error;
    float last_input; // Used for "Derivative on Measurement" to prevent setpoint kicks

    // Limits
    float max_integral;
    float max_output;
} PID_Controller;

typedef struct {
    PID_Controller roll;
    PID_Controller pitch;
    PID_Controller yaw;

    // Outer Angle Loop Gains
    float roll_angle_p;
    float pitch_angle_p;
} Flight_Control_t;

float PID_Compute(PID_Controller *pid, float target, float actual, float dt) ;
void PID_Reset(PID_Controller *pid, float current_sensor_value) ;


#endif /* INC_PID_CONTROL_H_ */
