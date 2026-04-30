#include "pid_control.h"



Flight_Control_t fc = {
    // Inner Rate Loops (The "Muscle")
		// Pitch needs more 'punch' (Kp) and 'brakes' (Kd) than Roll
		.roll  = {2.0f, 1.0f, 0.015f, 0.0f, 0.0f, 0.0f, 150.0f, 500.0f},
		.pitch = {2.0f, 1.0f, 0.015f, 0.0f, 0.0f, 0.0f, 150.0f, 500.0f},
		.yaw   = {4.00f, 2.00f, 0.000f, 0.0f, 0.0f, 0.0f, 150.0f, 500.0f},
    // Outer Angle Loops (The "Brain" for Leveling)
    .roll_angle_p  = 9.0f,
    .pitch_angle_p = 9.0f
};


float PID_Compute(PID_Controller *pid, float target, float actual, float dt) {
    // 1. Error calculation
    float error = target - actual;

    // 2. Proportional: Immediate response
    float p_term = pid->kp * error;

    // 3. Integral: Corrects long-term drift (Anti-windup included)
    pid->integral += (error * pid->ki) * dt;
    if (pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    else if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;

    // 4. Derivative: On Measurement (prevents 'D-kick' from stick snaps)
    // Formula: -Kd * (change in actual sensor data / time)
    float d_term = -pid->kd * (actual - pid->last_input) / dt;
    pid->last_input = actual;

    // 5. Combine and Constrain
    float output = p_term + pid->integral + d_term;
	if (output > pid->max_output)
    	output = pid->max_output;

    else if (output < -pid->max_output)
    	output = -pid->max_output;

    return output;
}

void PID_Reset(PID_Controller *pid, float current_sensor_value) {
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    // Prime the derivative state to match reality
    pid->last_input = current_sensor_value;


}
