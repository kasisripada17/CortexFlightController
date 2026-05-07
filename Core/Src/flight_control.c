/*
 * flight_control.c
 *
 *  Created on: 09-Apr-2026
 *      Author: kasiviswanadhsripada
 */
#include "flight_control.h"
#include "motors.h"
#include "radio.h"
#include "lsm6ds3.h"
#include "pid_control.h"
#include <stdbool.h>
#include "print.h"
#include <math.h>

// --- Altitude Hold Constants ---
#define VEL_Z_KP        12.0f    // Snappiness of vertical stop
#define VEL_Z_KI        0.8f     // Ability to hold weight over time
#define STICK_DEADZONE  50       // PWM deadzone around 1500 for DJI feel
#define MAX_CLIMB_RATE  2.5f     // Max vertical speed in m/s
#define MAX_RATE_ACRO 250.0f
#define MAX_TILT_ANGLE 35
uint8_t buffer[256];
extern float relative_altitude;
// Add these to your global variables or class members
bool alt_hold_initialized = false;
float locked_altitude = 0.0f;

uint16_t size;
bool is_angle_mode = false;
arm_state_t arm_status = DISARMED;
extern volatile receiver_t radio;
extern Flight_Control_t fc;
extern volatile IMU_Data_t sensor_data;
extern volatile uint8_t esc_calibration;
bool start_gyro_calibration = false;
extern Sensor_Calibration gyro_calibration;
extern float position_earth[3];
extern float velocity_earth[3];
bool motor_test = false;
#define PID_DT (1.0f / 416.0f)
static inline float constrain(float value, float min, float max) {
	if (value < min)
		return min;
	if (value > max)
		return max;
	return value;
}

Flight_Mode_t flight_mode = ACRO;

// Function Prototypes for visibility
void Mix_Motors_Adjusted(float r_cmd, float p_cmd, float y_cmd, float throttle);
void reset_inertial_nav(void); // Defined in motion_fx.c

void update_arm_status() {
	static uint32_t stick_hold_start = 0;

	bool sticks_in_disarm_position = (radio.throttle < 1100 && radio.yaw < 1100
			&& radio.pitch > 1900 && radio.roll > 1900);
	bool sticks_in_arm_position = (radio.throttle < 1100 && radio.yaw > 1900
			&& radio.pitch > 1900 && radio.roll < 1100);
	bool sticks_in_esc_calib_position = (radio.throttle < 1100
			&& radio.yaw < 1100 && radio.pitch > 1900 && radio.roll > 1400
			&& radio.roll < 1600);
	bool motor_test_pos = (radio.throttle > 1900 && radio.yaw < 1100
			&& radio.pitch > 1900 && (radio.roll > 1400 && radio.roll < 1600));
	bool armed_safe_pos = ((radio.throttle < 1100)
			&& (radio.yaw >= 1450 && radio.yaw <= 1550)
			&& (radio.pitch >= 1450 && radio.pitch <= 1550)
			&& (radio.roll >= 1450 && radio.roll <= 1550));

	switch (arm_status) {
	case DISARMED:
		if (sticks_in_arm_position) {
			stick_hold_start = HAL_GetTick();
			arm_status = ARMING;
		} else if (sticks_in_esc_calib_position == 1) {
			stick_hold_start = HAL_GetTick();
			arm_status = ESC_CALIBRATION;
		} else if (motor_test_pos == 1) {
			arm_status = MOTOR_TEST;
		}
		break;
	case ESC_CALIBRATION:

		if (sticks_in_esc_calib_position) {
			if (HAL_GetTick() - stick_hold_start >= 2000) {

				esc_calibration = 1;
			}
		}
		break;
	case ARMING:
		if (sticks_in_arm_position) {
			if (HAL_GetTick() - stick_hold_start >= 2000) {
				arm_status = ARMED_SAFE;
				// Reset PIDs
				fc.roll.integral = 0;
				fc.pitch.integral = 0;
				fc.yaw.integral = 0;

				fc.ground_offset = relative_altitude;

			}
		} else {
			arm_status = DISARMED;
		}
		break;

	case ARMED_SAFE:
		if (armed_safe_pos)
			arm_status = ARMED;
		break;

	case ARMED:
		if (sticks_in_disarm_position) {
			stick_hold_start = HAL_GetTick();
			arm_status = DISARMING;
		}
		break;

	case DISARMING:
		if (sticks_in_disarm_position) {
			if (HAL_GetTick() - stick_hold_start >= 2000)
				arm_status = DISARMED;
		} else {
			arm_status = ARMED;
		}
		break;
	default:
		break;
	}
}

void Mix_Motors_Adjusted(float r_cmd, float p_cmd, float y_cmd, float throttle) {
	float m[4];
	m[0] = throttle - p_cmd - r_cmd - y_cmd; // FR
	m[1] = throttle - p_cmd + r_cmd + y_cmd; // FL
	m[2] = throttle + p_cmd + r_cmd - y_cmd; // RL
	m[3] = throttle + p_cmd - r_cmd + y_cmd; // RR

	for (int i = 0; i < 4; i++) {
		if (m[i] < MOTOR_MIN)
			m[i] = MOTOR_MIN;
		if (m[i] > MOTOR_MAX)
			m[i] = MOTOR_MAX;
	}
	update_motors(m[0], m[1], m[2], m[3]);
}

void flight_control(void) {
	get_flight_mode();
	update_arm_status();
	const float dt = PID_DT;

	if (arm_status == ARMED) {
		float target_roll_rate = 0.0f, target_pitch_rate = 0.0f;
		float current_throttle = (float) radio.throttle;

		// --- Step 1: Altitude Management ---
		if (flight_mode == ALTITUDE_HOLD) {

			if (!alt_hold_initialized) {
				// LOCK THE HEIGHT: Capture current altitude as target
				fc.target_altitude = relative_altitude;

				// RESET PID: Clear previous I-term to prevent a sudden jump
				PID_Reset(&fc.alt, 0);

				alt_hold_initialized = 1;
			}

			current_throttle = compute_altitude_hold_throttle(dt);
			float target_roll_angle = normalize_radio(
					radio.roll) * MAX_TILT_ANGLE;
			float target_pitch_angle = -normalize_radio(
					radio.pitch) * MAX_TILT_ANGLE;

			target_roll_rate = (target_roll_angle - (-sensor_data.roll))
					* fc.roll_angle_p;
			target_pitch_rate = (target_pitch_angle - (-sensor_data.pitch))
					* fc.pitch_angle_p;
		}

		// --- Step 2: Attitude Management ---
		else if (flight_mode == ACRO) {
			alt_hold_initialized = 0;

			target_roll_rate = normalize_radio(radio.roll) * MAX_RATE_ACRO;
			target_pitch_rate = -normalize_radio(radio.pitch) * MAX_RATE_ACRO;
		} else if (flight_mode == SELF_LEVEL) {
			alt_hold_initialized = 0;

			float target_roll_angle = normalize_radio(
					radio.roll) * MAX_TILT_ANGLE;
			float target_pitch_angle = -normalize_radio(
					radio.pitch) * MAX_TILT_ANGLE;

			target_roll_rate = (target_roll_angle - (-sensor_data.roll))
					* fc.roll_angle_p;
			target_pitch_rate = (target_pitch_angle - (-sensor_data.pitch))
					* fc.pitch_angle_p;

		}

		// Clamp rates
		if (target_roll_rate > MAX_RATE_ACRO)
			target_roll_rate = MAX_RATE_ACRO;
		if (target_roll_rate < -MAX_RATE_ACRO)
			target_roll_rate = -MAX_RATE_ACRO;
		if (target_pitch_rate > MAX_RATE_ACRO)
			target_pitch_rate = MAX_RATE_ACRO;
		if (target_pitch_rate < -MAX_RATE_ACRO)
			target_pitch_rate = -MAX_RATE_ACRO;

		float target_yaw_rate = -normalize_radio(radio.yaw) * 300.0f;

		// --- Step 3: PID Computation ---
		float roll_cmd = PID_Compute(&fc.roll, target_roll_rate,
				sensor_data.gyro_cal_x, dt);
		float pitch_cmd = PID_Compute(&fc.pitch, target_pitch_rate,
				sensor_data.gyro_cal_y, dt);
		float yaw_cmd = PID_Compute(&fc.yaw, target_yaw_rate,
				sensor_data.gyro_cal_z, dt);

		// --- Step 4: Mixer Output ---
		Mix_Motors_Adjusted(roll_cmd, pitch_cmd, yaw_cmd, current_throttle);

	} else {
		alt_hold_initialized = 0;
		PID_Reset(&fc.roll, sensor_data.gyro_cal_x);
		PID_Reset(&fc.pitch, sensor_data.gyro_cal_y);
		PID_Reset(&fc.yaw, sensor_data.gyro_cal_z);
		if (motor_test == false && esc_calibration == false) {
			update_motors(MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF);
		}
	}


}


void get_flight_mode(void) {
	if (radio.mode >= 1000 && radio.mode <= 1300)
		flight_mode = ACRO;
	else if (radio.mode > 1300 && radio.mode <= 1700)
		flight_mode = SELF_LEVEL;
	else if (radio.mode > 1700 && radio.mode <= 2000)
		flight_mode = ALTITUDE_HOLD;
}


float compute_altitude_hold_throttle(float dt) {

	// 1. INITIALIZATION & SAFETY CHECK
	// If not armed or not in the right mode, reset and pass through manual throttle
	if (arm_status != ARMED) {
		alt_hold_initialized = false;
		return (float) radio.throttle;
	}

	// Standard Base Hover Throttle (adjust this for your drone's weight)
	const float HOVER_THROTTLE = 1500.0f;

	// Compute PID adjustment based on altitude error
	float adjustment = PID_Compute(&fc.alt, fc.target_altitude,
			relative_altitude, dt);

	float final_throttle = radio.throttle + adjustment;

	// Safety Clamps: Don't let Alt-Hold stop the motors or go full 100%
	if (final_throttle > 1800.0f)
		final_throttle = 1800.0f;
	if (final_throttle < 1200.0f)
		final_throttle = 1200.0f;

	return final_throttle;

}
