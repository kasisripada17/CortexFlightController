/*
 * altitude_hold.c
 *
 *  Created on: 17-May-2026
 *      Author: kasiviswanadhsripada
 */
#include "flight_control.h"
#include "stdbool.h"
#include "radio.h"
#include "pid_control.h"
#include "lsm6ds3.h"
#include "motors.h"
#include <stdio.h>
#include "print.h"
static float fused_alt = 0.0f;
static float fused_vel = 0.0f;
bool alt_hold_initialized = false;
float alt_fused = 0.0f;

extern arm_state_t armed_status ;
extern volatile receiver_t radio;
extern Flight_Control_t fc ;
extern volatile IMU_Data_t sensor_data ;
extern float ground_altitude;
extern float baro_altitude;
extern bool new_baro_ready ;
extern char buffer[256];
float acc_vel = 0.0f;
float acc_alt = 0.0f;
// Persistent filter state variable (initialize to 0.0f on startup or ground tare)
float low_pass_baro_alt = 0.0f;
extern float baroLPFAltitude;

float update_altitude_fusion(float dt) {
    // 1. --- IMU PREDICTION (Runs every cycle at 1666 Hz) ---
    fused_vel += sensor_data.acc_earth_z * dt;
    fused_alt += fused_vel * dt;

    // 2. --- BARO CORRECTION (Runs only when a new sensor sample arrives) ---
    if (new_baro_ready) {
        float baro_relative_alt = baroLPFAltitude - ground_altitude;

        // --- BAROMETER LOW-PASS FILTER (LPF) ---
        // BARO_ALPHA ranges from 0.0 to 1.0. Lower means smoother but adds lag.
        // 0.15f blocks high-frequency pressure spikes while preserving fast tracking.
        const float BARO_ALPHA = 0.15f;
        low_pass_baro_alt = (baro_relative_alt * BARO_ALPHA) + (low_pass_baro_alt * (1.0f - BARO_ALPHA));

        // Calculate tracking error based on the clean, filtered barometer data
        float position_error = low_pass_baro_alt - fused_alt;

        // --- COMPLEMENTARY FUSION TRACKING GAINS ---
        const float ALT_CORRECTION_GAIN = 0.15f;  // Snaps fused altitude to filtered baro
        const float VEL_CORRECTION_GAIN = 1.20f;  // Corrects IMU velocity integration drift

        // Apply corrections to our primary states
        fused_alt += position_error * ALT_CORRECTION_GAIN;
        fused_vel += position_error * VEL_CORRECTION_GAIN;

        // Clear flag until the next hardware read registers
        new_baro_ready = false;
    }

    // 3. --- SYSTEM DAMPING & OUTPUT ---
    fused_vel *= 0.9f;

    // Telemetry output for graph checking
    uint8_t size = sprintf(buffer, "%f, %f\r\n", fused_vel, fused_alt);
    usb_print(buffer, size);

    alt_fused = fused_alt;
    return fused_alt;
}

#ifdef TRADITIONAL_PWM_ESC
float compute_altitude_hold_throttle(float dt) {

	// 1. INITIALIZATION & SAFETY CHECK
	// If not armed or not in the right mode, reset and pass through manual throttle
	if (armed_status != ARMED) {
		alt_hold_initialized = false;
		return (float) radio.throttle;
	}

	// Standard Base Hover Throttle (adjust this for your drone's weight)

	// Compute PID adjustment based on altitude error
	float adjustment = PID_Compute(&fc.alt, fc.target_altitude, alt_fused, dt);

	float final_throttle = radio.throttle + adjustment;

	// Safety Clamps: Don't let Alt-Hold stop the motors or go full 100%
	if (final_throttle > 1800.0f)
		final_throttle = 1800.0f;
	if (final_throttle < 1200.0f)
		final_throttle = 1200.0f;

	return final_throttle;

}
#endif
#ifdef ONE_SHOT_ESCS
float compute_altitude_hold_throttle(float dt) {


     update_altitude_fusion( dt);

    // 2. COMPUTE PID ADJUSTMENT IN STANDARD PWM DOMAIN (1000-2000us)
    // Keeps your altitude loop gains consistent and independent of hardware timer frequencies
    float adjustment = PID_Compute(&fc.alt, fc.target_altitude, alt_fused, dt);

    // Combine stick input with altitude correction
    float final_throttle_pwm = (float)radio.throttle + adjustment;

    // 3. HARDWARE TICK CONVERSION
    // Scale standard 1000-2000us down to a 0.0 to 1.0 range, then map to tick window
    float throttle_percentage = (final_throttle_pwm - 1000.0f) / 1000.0f;

    // Safety protection against anomalous stick or PID inputs blowing out boundaries
    if (throttle_percentage > 1.0f) throttle_percentage = 1.0f;
    if (throttle_percentage < 0.0f) throttle_percentage = 0.0f;

    // Map to raw OneShot125 timer ticks
    float tick_range = (float)(ONESHOT125_MAX - ONESHOT125_MIN);
    float final_throttle_ticks = ONESHOT125_MIN + (throttle_percentage * tick_range);

    // 4. LOW-END AUTHORAuthority FLOOR (Idle Gap Enforcement)
    // Establishes a minimum floor so the Alt-Hold loop never chokes the motors below a spinning idle
    float absolute_floor = (float)ONESHOT125_MIN + MOTOR_IDLE_GAP;
    if (final_throttle_ticks < absolute_floor) {
        final_throttle_ticks = absolute_floor;
    }

    // 5. HIGH-END CEILING AUTHORITY CLAMP
    // Protects against full saturation so the mixer maintains headroom for roll/pitch adjustments
    // Enforces an 80% maximum ceiling in raw hardware ticks
    float absolute_ceiling = (float)ONESHOT125_MIN + (tick_range * 0.80f);
    if (final_throttle_ticks > absolute_ceiling) {
        final_throttle_ticks = absolute_ceiling;
    }

    return final_throttle_ticks;
}
#endif

