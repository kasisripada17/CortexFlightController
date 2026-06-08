
#include <stdint.h>
#include <stdio.h>
#include <math.h> // Mandated for standard isnan() and isinf() safety guards

#include "pid_control.h"
#include "print.h"



extern char buffer[256];
extern uint8_t size;



Flight_Control_t fc = {
	// Scale gains up so they have the authority to alter a 34,375-count timer scale
	.roll = {.kp = 150.0f, .ki = 10.0f, .kd = 5.0f, .max_i_output = 4500.0f, .max_output = 13750.0f, .d_cutoff_hz = 30.0f},
	.pitch = {.kp = 150.0f, .ki = 10.0f, .kd = 5.0f, .max_i_output = 4500.0f, .max_output = 13750.0f, .d_cutoff_hz = 30.0f},
	.yaw = {.kp = 500.0f, .ki = 25.0f, .kd = 0.0f, .max_i_output = 4500.0f, .max_output = 13750.0f,.d_cutoff_hz = 30.0f},
	.alt = {.kp = 60.0f, .ki = 8.0f, .kd = 30.0f, .max_i_output = 100.0f, .max_output = 250.0f,.d_cutoff_hz = 30.0f},

	//self level gains
	.roll_angle_p = 5.0f,
	.pitch_angle_p = 5.0f,
	.angle_mode_ki = 0.05f,
	.angle_mode_max_i = 15.0f,// Maximum corrective rate trim cap (degrees/second)

	//acro mode flight limits
	.max_rate = 300.0f,
	.max_tilt_angle = 50.0f
};


/**
 * @brief Computes the PID output for a single flight control axis.
 * @param pid    Pointer to the axis PID_Controller structure
 * @param target The desired target rate (or angle)
 * @param actual The current sensor measurement (from Gyro or IMU)
 * @param dt     The delta time since the last calculation loop (in seconds)
 * @return float The stabilized control output for the motor mixer
 */
float PID_Compute(PID_Controller *pid, float target, float actual, float dt) {
    // --- SAFETY GUARD 1: Prevent Jitter and Division-by-Zero ---
    // Clamps minimum dt to 100 microseconds to stop infinite spikes if interrupts overlap
    if (dt < 0.0001f) {
        dt = 0.0001f;
    }

    // 1. Error calculation
    float error = target - actual;

    // 2. Proportional Term (Immediate Correction)
    float p_term = pid->kp * error;

    // 3. Integral Tracking with Fixed Anti-Windup Clamping
    pid->integral += error * dt;

    // Uses your structure's actual boundary variable: max_i_output
    if (pid->integral > pid->max_i_output)  pid->integral = pid->max_i_output;
    if (pid->integral < -pid->max_i_output) pid->integral = -pid->max_i_output;

    float i_term = pid->ki * pid->integral;

    // 4. Derivative on Measurement (Dampens Acceleration, Avoids Stick-Kick)
    float gyro_delta = (actual - pid->last_input) / dt;
    pid->last_input = actual;

    // Protect against corrupt or broken hardware sensor reads
    if (isnan(gyro_delta) || isinf(gyro_delta)) {
        gyro_delta = 0.0f;
    }

    float raw_d_term = pid->kd * gyro_delta;

    // 5. Dynamic PT1 Low-Pass Filter Calculation
    // Uses your configured d_cutoff_hz (30.0f) to block high-frequency motor noise
    float tau = 1.0f / (2.0f * 3.14159265f * pid->d_cutoff_hz);
    float alpha = dt / (tau + dt);

    // Clamp filter smoothing factor strictly between 0.0 and 1.0
    if (alpha > 1.0f) alpha = 1.0f;
    if (alpha < 0.0f) alpha = 0.0f;

    // Apply the low-pass filter cleanly to isolate mechanical vibrations
    float d_term = pid->last_d_term + (alpha * (raw_d_term - pid->last_d_term));

    // Clear any broken floating-point values from contaminating future loops
    if (isnan(d_term) || isinf(d_term)) {
        d_term = 0.0f;
    }
    pid->last_d_term = d_term;

    // 6. Combine Terms (Standard subtraction dampening)
    // d_term acts as a brake opposing the rapid changes in acceleration
    float output = p_term + i_term - d_term;

    // 7. Master Constrain to Axis Authority Limits
    if (output > pid->max_output)       output = pid->max_output;
    else if (output < -pid->max_output) output = -pid->max_output;

    // --- SAFETY GUARD 2: Ultimate System Parachute ---
    // If anything bypassed the constraints, drop output to 0 to keep motors safe
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
