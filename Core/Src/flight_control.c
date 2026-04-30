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
uint8_t buffer[256];
uint16_t size;
bool is_angle_mode = true;
arm_state_t flight_mode = DISARMED;
extern receiver_t radio;
extern Flight_Control_t fc;
extern volatile IMU_Data_t sensor_data;
extern volatile uint8_t esc_calibration;
bool start_gyro_calibration = false;
extern Sensor_Calibration gyro_calibration;
bool motor_test = false;
#define PID_DT (1.0f / 416.0f)
extern float roll_copter;
extern float pitch_copter;
void update_arm_status() {
	static uint32_t stick_hold_start = 0;

	// Check for "Both Sticks Bottom-Center" position:
	// Left Stick: Bottom-Left (Throttle < 1100, Yaw < 1100)
	// Right Stick: Bottom-Right (Pitch < 1100, Roll > 1900)
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
	switch (flight_mode) {

	case DISARMED:
		usb_print(buffer, size);
		if (sticks_in_arm_position) {
			usb_print(buffer, size);
			stick_hold_start = HAL_GetTick();
			flight_mode = ARMING;
		} else if (sticks_in_esc_calib_position == 1) {
			usb_print(buffer, size);
			stick_hold_start = HAL_GetTick();
			flight_mode = ESC_CALIBRATION;

			usb_print((uint8_t*) "ESC_CALIBRATION_INIT\r\n",
					strlen("ESC_CALIBRATION_INIT\r\n"));
		} else if (motor_test_pos == 1) {
			flight_mode = MOTOR_TEST;

		}
		break;
	case MOTOR_TEST:
		motor_test = true;
		flight_mode = DISARMED;
		break;

	case ESC_CALIBRATION:
		if (sticks_in_esc_calib_position) {
			if (HAL_GetTick() - stick_hold_start >= 2000) {
				esc_calibration = 1;
			}
		} else {
			flight_mode = DISARMED; // Stick moved too soon
		}

		break;

	case ARMING:
		if (sticks_in_arm_position) {
			usb_print((uint8_t*) "ARMING\r\n", strlen("ARMING\r\n"));

			if (HAL_GetTick() - stick_hold_start >= 2000) { // 2 second hold

				usb_print((uint8_t*) "ARMING\r\n", strlen("ARMING\r\n"));
				flight_mode = ARMED_SAFE;
				// RESET PID INTEGRALS HERE so it doesn't jump on takeoff
				fc.roll.integral = 0;
				fc.pitch.integral = 0;
				fc.yaw.integral = 0;
			}
		} else {
			flight_mode = DISARMED; // Stick moved too soon
		}
		break;
	case ARMED_SAFE:
		// Check for Disarm: Same stick position
		if (armed_safe_pos) {
			flight_mode = ARMED;
		}
		break;
	case ARMED:
		// Check for Disarm: Same stick position
		if (sticks_in_disarm_position) {
			stick_hold_start = HAL_GetTick();
			flight_mode = DISARMING;
		}
		break;

	case DISARMING:

		if (sticks_in_disarm_position) {

			usb_print((uint8_t*) "DISARMING\r\n", strlen("DISARMING\r\n"));

			if (HAL_GetTick() - stick_hold_start >= 2000) {

				usb_print((uint8_t*) "DISARMED\r\n", strlen("DISARMED\r\n"));

				flight_mode = DISARMED;
			}
		} else {
			flight_mode = ARMED; // Stick moved too soon
		}
		break;
	}
}

void Mix_Motors(float r_cmd, float p_cmd, float y_cmd) {
	float m[4];

	// FINAL Mixer for Mapping: 0:FR, 1:FL, 2:RL, 3:RR
	// This version flips the pitch signs from your previous attempt.

	m[0] = radio.throttle - p_cmd - r_cmd - y_cmd; // Front Right (CCW)
	m[1] = radio.throttle - p_cmd + r_cmd + y_cmd; // Front Left (CW)
	m[2] = radio.throttle + p_cmd + r_cmd - y_cmd; // Rear Left (CCW)
	m[3] = radio.throttle + p_cmd - r_cmd + y_cmd; // Rear Right (CW)

	for (int i = 0; i < 4; i++) {
		if (m[i] < MOTOR_MIN) {
			m[i] = MOTOR_MIN;
		}
		// Idle speed
		if (m[i] > MOTOR_MAX) {
			m[i] = MOTOR_MAX;
		}
	}
	update_motors(m[0], m[1], m[2], m[3]);

}
// Define your max tilt for Angle Mode (e.g., 35 degrees)
#define MAX_TILT_ANGLE 30.0f
#define MAX_RATE_ACRO  250.0f

void flight_control(void) {

	update_arm_status();
	const float dt = PID_DT; // 416 hz

	if (flight_mode == ARMED) {

		float target_roll_rate = 0.0f, target_pitch_rate = 0.0f;

		// 1. Determine Rate Setpoints based on Mode
		if (is_angle_mode) {
			// OUTER LOOP: Convert stick position to Target Angle

			float target_roll_angle = normalize_radio(
					radio.roll) * MAX_TILT_ANGLE;
			float target_pitch_angle = -normalize_radio(
					radio.pitch) * MAX_TILT_ANGLE;

			// 2. Calculate Error (Target - Current)
			float error_roll = target_roll_angle - (-sensor_data.roll);
			float error_pitch = target_pitch_angle - (-sensor_data.pitch);

			target_roll_rate = (error_roll * fc.roll_angle_p);
			target_pitch_rate = (error_pitch * fc.pitch_angle_p);
			// Clamp the commanded rate so it doesn't exceed what the inner loop can handle
			// 4. Rate Limiting (Keep the drone within its mechanical limits)
			if (target_roll_rate > MAX_RATE_ACRO)
				target_roll_rate = MAX_RATE_ACRO;
			if (target_roll_rate < -MAX_RATE_ACRO)
				target_roll_rate = -MAX_RATE_ACRO;
			if (target_pitch_rate > MAX_RATE_ACRO)
				target_pitch_rate = MAX_RATE_ACRO;
			if (target_pitch_rate < -MAX_RATE_ACRO)
				target_pitch_rate = -MAX_RATE_ACRO;
		} else {
			// ACRO MODE: Sticks directly control rotation rate
			target_roll_rate = normalize_radio(radio.roll) * MAX_RATE_ACRO;
			target_pitch_rate = -normalize_radio(radio.pitch) * MAX_RATE_ACRO;
		}

		// Yaw usually stays in Rate mode even in Angle mode
		float target_yaw_rate = -normalize_radio(radio.yaw) * 300.0f;

		// 2. INNER LOOP: Compute Rate PID outputs
		// This math is the same regardless of how the target_rate was calculated
		float roll_cmd = PID_Compute(&fc.roll, target_roll_rate,
				sensor_data.gyro_cal_x, dt);
		float pitch_cmd = PID_Compute(&fc.pitch, target_pitch_rate,
				sensor_data.gyro_cal_y, dt);
		float yaw_cmd = PID_Compute(&fc.yaw, target_yaw_rate,
				sensor_data.gyro_cal_z, dt);

//
//        size = sprintf((char*)buffer,"\r\n%f,%f,%f,%f,%f,%f",target_roll_rate, target_pitch_rate,target_yaw_rate,roll_cmd,pitch_cmd, yaw_cmd);
//        usb_print(buffer,size);
		// 3. Final Mixing (Only called ONCE)
		Mix_Motors(roll_cmd, pitch_cmd, yaw_cmd);

	} else {
		if (motor_test == false && esc_calibration == false) {
			update_motors(MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF);
		}
		// Important: Reset PID integrals when disarmed to prevent "windup" on ground
		PID_Reset(&fc.roll, sensor_data.gyro_cal_x);
		PID_Reset(&fc.pitch, sensor_data.gyro_cal_y);
		PID_Reset(&fc.yaw, sensor_data.gyro_cal_z);
		return;
	}
}
