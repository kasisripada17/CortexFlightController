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

arm_state_t flight_mode = DISARMED;
extern receiver_t radio;
extern Flight_Control_t fc ;
extern volatile IMU_Data_t sensor_data;

void update_arm_status() {
    static uint32_t stick_hold_start = 0;

    // Check for "Both Sticks Bottom-Center" position:
    // Left Stick: Bottom-Left (Throttle < 1100, Yaw < 1100)
    // Right Stick: Bottom-Right (Pitch < 1100, Roll > 1900)
    bool sticks_in_arm_position = (radio.throttle < 1100 && radio.yaw < 1100 &&
                                   radio.pitch < 1100    && radio.roll > 1900);

    switch (flight_mode) {
        case DISARMED:
            if (sticks_in_arm_position) {
                stick_hold_start = HAL_GetTick();
                flight_mode = ARMING;
            }
            break;

        case ARMING:
            if (sticks_in_arm_position) {
                if (HAL_GetTick() - stick_hold_start >= 2000) { // 2 second hold
                    flight_mode = ARMED;
                    // RESET PID INTEGRALS HERE so it doesn't jump on takeoff
                    fc.roll.integral = 0; fc.pitch.integral = 0; fc.yaw.integral = 0;
                }
            } else {
                flight_mode = DISARMED; // Stick moved too soon
            }
            break;

        case ARMED:
            // Check for Disarm: Same stick position
            if (sticks_in_arm_position) {
                stick_hold_start = HAL_GetTick();
                flight_mode = DISARMING;
            }
            break;

        case DISARMING:
            if (sticks_in_arm_position) {
                if (HAL_GetTick() - stick_hold_start >= 2000) {
                    flight_mode = DISARMED;
                }
            } else {
                flight_mode = ARMED; // Stick moved too soon
            }
            break;
    }
}

void Mix_Motors( float r_cmd, float p_cmd, float y_cmd) {
    float m[4];

    // Kill motors immediately if not ARMED
    if (flight_mode != ARMED) {
        	update_motors(MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF); // Send stop pulse
        return;
    }

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
	update_motors(m[0],m[1],m[2],m[3]);

}

void flight_control(void) {

	const float dt = 0.0006024f; // 1.66 kHz period

	float target_roll = normalize_radio(radio.roll) * 250.0f;
	float target_pitch = normalize_radio(radio.pitch) * 250.0f;
	float target_yaw = normalize_radio(radio.yaw) * 300.0f;

	// 2. Compute PID outputs
	float roll_cmd = PID_Compute(&fc.roll, target_roll, sensor_data.gyro_x, dt);
	float pitch_cmd = PID_Compute(&fc.pitch, target_pitch, sensor_data.gyro_y, dt);
	float yaw_cmd = PID_Compute(&fc.yaw, target_yaw, sensor_data.gyro_z, dt);
	 Mix_Motors(  roll_cmd,  pitch_cmd,  yaw_cmd) ;


}
