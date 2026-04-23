#include "pid_control.h"



Flight_Control_t fc = {
    // Inner Rate Loops (The "Muscle")
    .roll  = {0.15f, 0.05f, 0.003f, 0.0f, 0.0f, 0.0f, 400.0f, 500.0f},
    .pitch = {0.15f, 0.05f, 0.003f, 0.0f, 0.0f, 0.0f, 400.0f, 500.0f},
    .yaw   = {0.25f, 0.10f, 0.001f, 0.0f, 0.0f, 0.0f, 400.0f, 500.0f},

    // Outer Angle Loops (The "Brain" for Leveling)
    .roll_angle_p  = 4.5f,
    .pitch_angle_p = 4.5f
};

const float dt = 0.0006024f; // 1.66 kHz period

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

void PID_Reset(PID_Controller *pid) {
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last_input = 0.0f; // Resetting this prevents a "D-term kick" on the first loop
}


