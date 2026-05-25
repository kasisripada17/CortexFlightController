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
		        .kp = 0.7f, .ki = 0.2f, .kd = 0.004f, .kff = 0.15f,
		        .max_i_output = 150.00f, .max_output = 400.0f, .d_cutoff_hz = 20.0f,
		        .integral = 0.0f, .last_error = 0.0f, .last_input = 0.0f,
		        .last_d_term = 0.0f, .last_target = 0.0f, .d_filter_alpha = 0.0f, .output = 0.0f
		    },
		    .pitch = {
		        // With the 0.80f mixer fix, KP can align at 1.05f to account purely for lower inertia.
		        // KD is dropped by 20% to 0.0032f because there is no weight on the nose/tail to dampen.
		        .kp = 0.75f, .ki = 0.2f, .kd = 0.0026f, .kff = 0.15f,
		        .max_i_output = 150.00f, .max_output = 400.0f, .d_cutoff_hz = 20.0f,
		        .integral = 0.0f, .last_error = 0.0f, .last_input = 0.0f,
		        .last_d_term = 0.0f, .last_target = 0.0f, .d_filter_alpha = 0.0f, .output = 0.0f
		    },
		    .yaw   = {
		        // Tuned to prevent the low-inertia pitch axis from dipping during high-speed spins
		        .kp = 0.75f, .ki = 0.12f, .kd = 0.0012f, .kff = 0.05f,
		        .max_i_output = 150.00f, .max_output = 400.0f, .d_cutoff_hz = 20.0f,
		        .integral = 0.0f, .last_error = 0.0f, .last_input = 0.0f,
		        .last_d_term = 0.0f, .last_target = 0.0f, .d_filter_alpha = 0.0f, .output = 0.0f
		    },
    // Outer Angle Loops (The "Brain" for Leveling)
    .roll_angle_p  = 3.5f,
    .pitch_angle_p = 3.5f,

    // Altitude Management
    .target_altitude = 0.0f,
    .hover_throttle  = 1500.0f, // Example baseline PWM/digital throttle value
    .ground_offset   = 0.0f,

    .target_roll_rate  = 0.0f,
    .target_pitch_rate = 0.0f,
    .target_yaw_rate   = 0.0f
};
#include <math.h> // Mandated for standard isnan() and isinf() safety guards

float PID_Compute(PID_Controller *pid, float target, float actual, float dt) {
    // --- SAFETY GUARD 1: Strict Timing Lower Bound ---
    // Prevents extreme division spikes if an interrupt fires early or jitter occurs
    // Assuming 1.66kHz target loop time (0.0006s). Clamp minimum dt to 100 microseconds.
    if (dt < 0.0001f) {
        dt = 0.0006f; // Safe fallback to standard loop time constant
    }

    // 1. Error calculation
    float error = target - actual;

    // 2. Proportional term
    float p_term = pid->kp * error;

    // 3. Clean Integral tracking with Dynamic Back-Calculation Anti-Windup
    pid->integral += error * dt;
    // Clamp the accumulator directly using pre-calculated bounds
            if (pid->integral > pid->max_i_output)  pid->integral = pid->max_i_output;
            if (pid->integral < -pid->max_i_output) pid->integral = -pid->max_i_output;
    float i_term = pid->ki * pid->integral;
//
//    if (pid->ki > 0.0001f) {
//        if (i_term > pid->max_i_output) {
//            i_term = pid->max_i_output;
//            pid->integral = i_term / pid->ki;
//        }
//        else if (i_term < -pid->max_i_output) {
//            i_term = -pid->max_i_output;
//            pid->integral = i_term / pid->ki;
//        }
//    } else {
//        i_term = 0.0f;
//        pid->integral = 0.0f;
//    }

    // 4. Hardened Derivative on Measurement
    float gyro_delta = (actual - pid->last_input) / dt;
    pid->last_input = actual;

    // --- SAFETY GUARD 2: Clip Infinite D-Term Spikes ---
    if (isnan(gyro_delta) || isinf(gyro_delta)) {
        gyro_delta = 0.0f;
    }

    float raw_d_term = pid->kd * gyro_delta;

    // Dynamic PT1 Filter calculation
    float tau = 1.0f / (2.0f * 3.14159265f * pid->d_cutoff_hz);
    float alpha = dt / (tau + dt);

    if (alpha > 1.0f) alpha = 1.0f;
    if (alpha < 0.0f) alpha = 0.0f;

    // Apply low-pass filter cleanly
    float d_term = pid->last_d_term + (alpha * (raw_d_term - pid->last_d_term));

    // --- SAFETY GUARD 3: Purge PT1 Filter Contamination ---
    if (isnan(d_term) || isinf(d_term)) {
        d_term = 0.0f;
    }
    pid->last_d_term = d_term;

    // 5. Hardened Feed-Forward on stick velocity
    float target_delta = (target - pid->last_target) / dt;
    pid->last_target = target;

    // --- SAFETY GUARD 4: Clip Infinite Stick Delts ---
    if (isnan(target_delta) || isinf(target_delta)) {
        target_delta = 0.0f;
    }
    float ff_term = pid->kff * target_delta;

    // 6. Combine terms (Standard subtraction dampening)
    float output = p_term + i_term - d_term + ff_term;

    // 7. Master constrain to full control authority limits
    if (output > pid->max_output)
        output = pid->max_output;
    else if (output < -pid->max_output)
        output = -pid->max_output;

    // --- SAFETY GUARD 5: Ultimate System Parachute ---
    // If anything bypassed the system and corrupted the output, drop it to zero
    // to keep the downstream hardware timers running safely.
    if (isnan(output) || isinf(output)) {
        return 0.0f;
    }

    return output;
}
void PID_Reset(PID_Controller *pid, float current_sensor_value) {
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    // Prime the derivative state to match reality
    pid->last_input = current_sensor_value;


}
