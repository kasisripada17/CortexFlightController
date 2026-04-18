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

arm_state_t flight_mode = DISARMED;
extern receiver_t radio;
extern Flight_Control_t fc;
extern volatile IMU_Data_t sensor_data;
extern uint8_t esc_calibration;
bool start_gyro_calibration = false;
extern Sensor_Calibration gyro_calibration;

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
	bool sticks_in_gyro_force_calib = (radio.throttle > 1900 && radio.yaw < 1100
			&& radio.pitch > 1900 && (radio.roll > 1400 && radio.roll < 1600));
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

		usb_print((uint8_t*)"ESC_CALIBRATION_INIT\r\n", strlen("ESC_CALIBRATION_INIT\r\n"));
	} else if (sticks_in_gyro_force_calib == 1) {
		flight_mode = GYRO_CALIBRATION;
		start_gyro_calibration = true;

	}
	break;
case GYRO_CALIBRATION:
	if (gyro_calibration == CALIBRATED) {
		flight_mode = DISARMED;
	}
	break;

case ESC_CALIBRATION:
	if (sticks_in_esc_calib_position) {
		if (HAL_GetTick() - stick_hold_start >= 2000) {

			usb_print((uint8_t*)"ESC_CALIBRATION_STARETED\r\n", strlen("ESC_CALIBRATION_STARETED\r\n"));

			usb_print(buffer, size);
			esc_calibration = 1;
		}
	} else {
		flight_mode = DISARMED; // Stick moved too soon
	}

	break;

case ARMING:
	if (sticks_in_arm_position) {
		if (HAL_GetTick() - stick_hold_start >= 2000 && (gyro_calibration == CALIBRATED)) { // 2 second hold

			usb_print((uint8_t*)"ARMED\r\n", strlen("ARMED\r\n"));
			flight_mode = ARMED;
			// RESET PID INTEGRALS HERE so it doesn't jump on takeoff
			fc.roll.integral = 0;
			fc.pitch.integral = 0;
			fc.yaw.integral = 0;
		}
	} else {
		flight_mode = DISARMED; // Stick moved too soon
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

		usb_print(buffer, size);

		if (HAL_GetTick() - stick_hold_start >= 2000) {

			usb_print((uint8_t*)"DISARMED\r\n", strlen("DISARMED\r\n"));

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

	// Standard Quad-X Mix
	m[0] = radio.throttle - p_cmd - r_cmd + y_cmd; // Front Right
	m[1] = radio.throttle + p_cmd - r_cmd - y_cmd; // Rear Right
	m[2] = radio.throttle + p_cmd + r_cmd + y_cmd; // Rear Left
	m[3] = radio.throttle - p_cmd + r_cmd - y_cmd; // Front Left

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

void flight_control(void) {

	update_arm_status();
	const float dt = 0.0006024f; // 1.66 kHz period
	if (flight_mode == ARMED) {
		float target_roll = normalize_radio(radio.roll) * 250.0f;
		float target_pitch = normalize_radio(radio.pitch) * 250.0f;
		float target_yaw = normalize_radio(radio.yaw) * 300.0f;

		// 2. Compute PID outputs
		float roll_cmd = PID_Compute(&fc.roll, target_roll, sensor_data.gyro_x,
				dt);
		float pitch_cmd = PID_Compute(&fc.pitch, target_pitch,
				sensor_data.gyro_y, dt);
		float yaw_cmd = PID_Compute(&fc.yaw, target_yaw, sensor_data.gyro_z,
				dt);
		Mix_Motors(roll_cmd, pitch_cmd, yaw_cmd);

	} else if (flight_mode != ARMED) {
		update_motors(MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF); // Send stop pulse
		return;
	}

}
