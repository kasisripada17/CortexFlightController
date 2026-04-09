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

// Container for all 3 axes
typedef struct {
	PID_Controller roll;
	PID_Controller pitch;
	PID_Controller yaw;
} Flight_Control_t;

float PID_Compute(PID_Controller *pid, float target, float actual, float dt) ;


#endif /* INC_PID_CONTROL_H_ */
