/*
 * pid_control.h
 *
 *  Created on: 07-Apr-2026
 *      Author: kasiviswanadhsripada
 */

#ifndef INC_PID_CONTROL_H_
#define INC_PID_CONTROL_H_

#define PID_DT DT

typedef struct {
    // Gains
    float kp;
    float ki;
    float kd;
    float kff;               // <--- Added: Feed-Forward Gain

    // State variables
    float integral;
    float last_error;
    float last_input;        // Used for "Derivative on Measurement"
    float last_d_term;       // <--- Added: State tracker for the PT1 filter
    float last_target;

    // Filtering
    float d_filter_alpha;    // <--- Added: Cutoff weight (0.1 to 0.5)

    // Limits
    float max_i_output;
    float max_output;
    float output;
    float d_cutoff_hz;  // Set this to 20.0f or 30.0f Hz instead of a raw alpha

} PID_Controller;




typedef struct {
	PID_Controller roll;
	PID_Controller pitch;
	PID_Controller yaw;
	PID_Controller alt;      // <--- Added for Altitude Hold
	// Outer Angle Loop Gains
	const float roll_angle_p;
	const float pitch_angle_p;
	const float angle_mode_ki;
	const float angle_mode_max_i;
	// Altitude Management
	float target_altitude;   // The "locked" height
	float hover_throttle;    // The base throttle needed to stay level
	float ground_offset;
	//attitude mode variables
	float target_roll_rate;
	float target_pitch_rate;
	float target_yaw_rate;
	//flight limits
	const float max_rate;
	const float max_tilt_angle;

} Flight_Control_t;

float PID_Compute(PID_Controller *pid, float target, float actual, float dt) ;
void PID_Reset(PID_Controller *pid, float current_sensor_value) ;


#endif /* INC_PID_CONTROL_H_ */
