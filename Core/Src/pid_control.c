#include "pid_control.h"
#include "print.h"
#include <stdint.h>
#include <stdio.h>
extern uint8_t buffer[256];
extern uint8_t size;

Flight_Control_t fc = {
// Inner Rate Loops (The "Muscle")
// Format: {kp, ki, kd, kff, integral, last_error, last_input, last_d_term, d_filter_alpha, max_integral, max_output, output, d_cutoff_hz}


		.roll  = {
		        .kp = 0.4f, .ki = 0.1f, .kd = 0.008f, .kff = 0.15f,
		        .max_i_output =  150.00f, .max_output = 400.0f, .d_cutoff_hz = 30.0f,
		        .integral = 0.0f, .last_error = 0.0f, .last_input = 0.0f,
		        .last_d_term = 0.0f, .last_target = 0.0f, .d_filter_alpha = 0.0f, .output = 0.0f
		    },
		    .pitch = {
		        .kp = 0.4f, .ki = 0.1f, .kd = 0.008f, .kff = 0.15f,
		        .max_i_output = 150.00f, .max_output = 400.0f, .d_cutoff_hz = 30.0f,
		        .integral = 0.0f, .last_error = 0.0f, .last_input = 0.0f,
		        .last_d_term = 0.0f, .last_target = 0.0f, .d_filter_alpha = 0.0f, .output = 0.0f
		    },
		    .yaw   = {
		        .kp = 0.75f, .ki = 0.3f, .kd = 0.000f, .kff = 0.00f,
		        .max_i_output = 150.00f, .max_output = 400.0f, .d_cutoff_hz = 20.0f,
		        .integral = 0.0f, .last_error = 0.0f, .last_input = 0.0f,
		        .last_d_term = 0.0f, .last_target = 0.0f, .d_filter_alpha = 0.0f, .output = 0.0f
		    },
    // Outer Angle Loops (The "Brain" for Leveling)
    .roll_angle_p  = 2.5f,
    .pitch_angle_p = 2.5f,

    // Altitude Management
    .target_altitude = 0.0f,
    .hover_throttle  = 1500.0f, // Example baseline PWM/digital throttle value
    .ground_offset   = 0.0f,

    .target_roll_rate  = 0.0f,
    .target_pitch_rate = 0.0f,
    .target_yaw_rate   = 0.0f
};
float PID_Compute(PID_Controller *pid, float target, float actual, float dt) {
    if (dt <= 0.0f) return 0.0f;

    // 1. Error calculation (-1.0 to 1.0 scale)
    float error = target - actual;

    // 2. Proportional term
    float p_term = pid->kp * error;

    // 3. Clean Integral tracking (Accumulate raw error * dt)
    pid->integral += error * dt;


    float i_term = pid->ki * pid->integral;
    // Smooth anti-windup clamp on the raw integrator register
//    if (pid->integral > pid->max_integral)       pid->integral = pid->max_integral;
//    else if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;


    if (pid->ki > 0.0001f) {
        // Upper limit clamp and back-calculation
        if (i_term > pid->max_i_output) {
            i_term = pid->max_i_output;
            pid->integral = i_term / pid->ki;
        }
        // Lower limit clamp and back-calculation
        else if (i_term < -pid->max_i_output) {
            i_term = -pid->max_i_output;
            pid->integral = i_term / pid->ki;
        }
    } else {
        // If Ki is 0, completely clear the terms to be safe
        i_term = 0.0f;
        pid->integral = 0.0f;
    }
    // 4. Derivative on Measurement (with proper normalized delta scaling)
    // Division by dt scales frame delta to 'units per second' fraction
    float gyro_delta = (actual - pid->last_input) / dt;
    pid->last_input = actual;

    float raw_d_term = pid->kd * gyro_delta;

    // Dynamic PT1 Filter calculation
    float tau = 1.0f / (2.0f * 3.14159265f * pid->d_cutoff_hz);
    float alpha = dt / (tau + dt);

    if (alpha > 1.0f) alpha = 1.0f;
    if (alpha < 0.0f) alpha = 0.0f;

    // Apply low-pass filter cleanly
    float d_term = pid->last_d_term + (alpha * (raw_d_term - pid->last_d_term));
    pid->last_d_term = d_term;

    // 5. Feed-Forward on stick velocity
    float target_delta = (target - pid->last_target) / dt;
    float ff_term = pid->kff * target_delta;
    pid->last_target = target;

    // 6. Combine terms
    // i_term is now cleanly isolated, d_term correctly dampens
    float output = p_term + i_term - d_term + ff_term;

    // 7. Master constrain to full control authority limits
    if (output > pid->max_output)        output = pid->max_output;
    else if (output < -pid->max_output) output = -pid->max_output;

    return output;
}
void PID_Reset(PID_Controller *pid, float current_sensor_value) {
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    // Prime the derivative state to match reality
    pid->last_input = current_sensor_value;


}
