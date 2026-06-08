/*
 * flight_control.c
 *
 *  Created on: 09-Apr-2026
 *      Author: kasiviswanadhsripada
 */
#include <stdbool.h>
#include <math.h>
#include <stdbool.h>
//#include "arm_math.h"

#include "flight_control.h"
#include "motors.h"
#include "radio.h"
#include "lsm6ds3.h"
#include "pid_control.h"
#include "print.h"
#include "altitude_hold.h"



#define TARGET_CENTER  (0.50f)
#define DEADBAND_HALF_WIDTH  (0.1f)
#define MAX_CLIMB_RATE  (2.5f)

extern float baroLPFAltitude ;

//variables for altitude hold
float deadband_lower = TARGET_CENTER - DEADBAND_HALF_WIDTH;
float deadband_upper = TARGET_CENTER + DEADBAND_HALF_WIDTH;
bool is_alt_locked = false;






char buffer[256];
extern float relative_altitude;
extern float alt_fused;
extern bool alt_hold_initialized;
extern bool new_baro_ready;
float outer_roll_integral = 0.0f;
float outer_pitch_integral = 0.0f;
float ground_altitude  = 0.0f;
extern float baro_altitude;





//#define TPA_BREAKPOINT 1350.0f  // Throttle level where attenuation begins (just above hover)


// --- Global Compile Definitions ---
#define BRAKING_FORCE 3.20f      // Tuning // Configuration constants

// --- Structure and Persistence Layer ---
typedef struct {
	float last_stick_position;   // Tracks stick position over time frames
} Braking_State_t;

// Persistent tracking states for stick movement velocity
static Braking_State_t brake_roll = { 0.0f };
static Braking_State_t brake_pitch = { 0.0f };

// Prototypes to ensure the compiler maps execution correctly
float Calculate_Braking_Target_Angle(float current_stick, float max_tilt_angle,
		Braking_State_t *state, float dt);

// --- Step 3: PID Computation ---
float roll_cmd = 0.0f;
float pitch_cmd = 0.0f;
float yaw_cmd = 0.0f;

// Add these to your global variables or class members
float locked_altitude = 0.0f;
extern float a_global[3];
uint16_t size;
bool is_angle_mode = false;
arm_state_t armed_status = DISARMED;
extern volatile receiver_t radio;
extern Flight_Control_t fc;
extern volatile IMU_Data_t sensor_data;
extern volatile uint8_t esc_calibration;
bool start_gyro_calibration = false;
extern Sensor_Calibration gyro_calibration;
extern float velocity_earth[3];
bool motor_test = false;

static inline float constrain(float value, float min, float max) {
	if (value < min)
		return min;
	if (value > max)
		return max;
	return value;
}

float gyro_filteredx = 0.0f, gyro_filteredy = 0.0f, gyro_filteredz = 0.0f;

Flight_Mode_t flight_mode = ACRO;
extern float accx, accy, accz;
extern float angx, angy, angz;
// Function Prototypes for visibility
void Mix_Motors_Adjusted(float r_cmd, float p_cmd, float y_cmd, float throttle);


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

	switch (armed_status) {
	case DISARMED:
		if (sticks_in_arm_position) {
			stick_hold_start = HAL_GetTick();
			armed_status = ARMING;
		} else if (sticks_in_esc_calib_position == 1) {
			stick_hold_start = HAL_GetTick();
			armed_status = ESC_CALIBRATION;
		} else if (motor_test_pos == 1) {
			armed_status = MOTOR_TEST;
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
			if (HAL_GetTick() - stick_hold_start >= 1000) {
				armed_status = ARMED_SAFE;
				// Reset PIDs
				fc.roll.integral = 0.0f;
				fc.pitch.integral = 0.0f;
				fc.yaw.integral = 0.0f;
				ground_altitude = baroLPFAltitude;

			}
		} else {
			armed_status = DISARMED;
		}
		break;

	case ARMED_SAFE:
		if (armed_safe_pos)
			armed_status = ARMED;
		break;

	case ARMED:
		if (sticks_in_disarm_position) {
			stick_hold_start = HAL_GetTick();
			armed_status = DISARMING;

		}
		break;

	case DISARMING:
		if (sticks_in_disarm_position) {
			if (HAL_GetTick() - stick_hold_start >= 1000)
				armed_status = DISARMED;
		} else {
			armed_status = ARMED;
		}
		break;
	default:
		break;
	}
}

#ifdef ONE_SHOT_ESCS
void Mix_Motors_Adjusted(float r_cmd, float p_cmd, float y_cmd, float throttle) {
		float m[4];
		m[0] = throttle - p_cmd - r_cmd - y_cmd; // FR
		m[1] = throttle - p_cmd + r_cmd + y_cmd; // FL
		m[2] = throttle + p_cmd + r_cmd - y_cmd; // RL
		m[3] = throttle + p_cmd - r_cmd + y_cmd; // RR
	// 2. Hardware Bounds Clamp Protection




		for (int i = 0; i < 4; i++) {
			if (m[i] < ONESHOT125_MIN)
				m[i] = ONESHOT125_MIN;
			if (m[i] > ONESHOT125_MAX)
				m[i] = ONESHOT125_MAX;
		}

		update_motors(m[0], m[1], m[2], m[3]);

}
#endif
#ifdef TRADITION_PWM_ESCS
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
#endif



/**
 * @brief Main flight control loop task executed at the system rate (PID_DT).
 */
void flight_control(void) {
	// 1. Start profiling latency immediately
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

	get_flight_mode();
	update_arm_status();
//	update_tuning_from_radio();
	const float dt = PID_DT;

	// Fix 1: State Machine Housekeeping
	if (flight_mode != ALTITUDE_HOLD) {
		alt_hold_initialized = 0U;
	}

	if (armed_status == ARMED) {


		//normalize throttle from 1000us-2000us pulses to 0.0f to 1.0f
		float throttle_scalar = normalize_radio_throttle(radio.throttle);

		// Map smoothly starting directly from your ARMED_IDLE up to MAX across full stick travel
		float current_throttle =  ONESHOT125_MIN+ (throttle_scalar * (ONESHOT125_MAX-ONESHOT125_MIN))+MOTOR_IDLE_GAP;

		// Normalize radio sticks (-1.0 to 1.0)
		float stick_roll = normalize_radio(radio.roll);
		float stick_pitch = -normalize_radio(radio.pitch);
		float stick_yaw = -normalize_radio(radio.yaw);

		// Default: Set target rates directly to ACRO capabilities
		fc.target_roll_rate = stick_roll * fc.max_rate;
		fc.target_pitch_rate = stick_pitch * fc.max_rate;
		fc.target_yaw_rate = stick_yaw * fc.max_rate;

		// Cascade Attitude Controller (Self-Level / Altitude Hold)
		if (flight_mode == SELF_LEVEL || flight_mode == ALTITUDE_HOLD) {

			if (flight_mode == ALTITUDE_HOLD) {
				if (!alt_hold_initialized) {
					PID_Reset(&fc.alt, 0);          // Flush Z-axis I-term
					alt_hold_initialized = 1U;
				}

				if (throttle_scalar >= deadband_lower && throttle_scalar <= deadband_upper) {

					if (!is_alt_locked) {
						fc.target_altitude = alt_fused; // Lock height
						is_alt_locked = true; // Latch the state so it doesn't overwrite next cycle
						size  = sprintf(buffer,"altitude updated %f\r\n",fc.target_altitude);
						usb_print(buffer,size);

					}
				}else
				{
					is_alt_locked = false;
				}

				current_throttle = compute_altitude_hold_throttle(dt);


			}

			// -----------------------------------------------------------------
			// INTEGRATED BRAKING TARGET CALLS
			// -----------------------------------------------------------------
			// We pass stick inputs, constraints, persistent track structs, and dt
//            float target_angle_roll  = Calculate_Braking_Target_Angle(stick_roll,  MAX_TILT_ANGLE, &brake_roll,  dt);
//            float target_angle_pitch = Calculate_Braking_Target_Angle(stick_pitch, MAX_TILT_ANGLE, &brake_pitch, dt);
			// -----------------------------------------------------------------

			float target_angle_roll = stick_roll * fc.max_tilt_angle;
			float target_angle_pitch = stick_pitch * fc.max_tilt_angle;

			// 1. Calculate raw angle error differences
			float angle_error_roll = target_angle_roll - sensor_data.roll;
			float angle_error_pitch = target_angle_pitch - sensor_data.pitch;

			// 2. Accumulate historical errors to counter persistent drift (Trim Engine)
			outer_roll_integral += angle_error_roll * dt;
			outer_pitch_integral += angle_error_pitch * dt;

			// 3. Smooth Anti-Windup Clamps to protect your control bounds
			if (outer_roll_integral > fc.angle_mode_max_i)
				outer_roll_integral = fc.angle_mode_max_i;
			if (outer_roll_integral < -fc.angle_mode_max_i)
				outer_roll_integral = -fc.angle_mode_max_i;
			if (outer_pitch_integral > fc.angle_mode_max_i)
				outer_pitch_integral = fc.angle_mode_max_i;
			if (outer_pitch_integral < -fc.angle_mode_max_i)
				outer_pitch_integral = -fc.angle_mode_max_i;

			// 4. Compute composite error targets combining Proportional and Integral blocks
			float error_rate_roll = (angle_error_roll * fc.roll_angle_p)
					+ (outer_roll_integral * fc.angle_mode_ki);
			float error_rate_pitch = (angle_error_pitch * fc.pitch_angle_p)
					+ (outer_pitch_integral * fc.angle_mode_ki);

			fc.target_roll_rate = error_rate_roll;
			fc.target_pitch_rate = error_rate_pitch;

		}

		// Global Inner-Loop Rate Clamping
		if (fc.target_roll_rate > fc.max_rate)
			fc.target_roll_rate = fc.max_rate;
		if (fc.target_roll_rate < -fc.max_rate)
			fc.target_roll_rate = -fc.max_rate;
		if (fc.target_pitch_rate > fc.max_rate)
			fc.target_pitch_rate = fc.max_rate;
		if (fc.target_pitch_rate < -fc.max_rate)
			fc.target_pitch_rate = -fc.max_rate;
		if (fc.target_yaw_rate > fc.max_rate)
			fc.target_yaw_rate = fc.max_rate;
		if (fc.target_yaw_rate < -fc.max_rate)
			fc.target_yaw_rate = -fc.max_rate;


		float tpa_factor = 1.0f;
		if (current_throttle > TPA_BREAKPOINT) {
			float throttle_range = TPA_MAX_LIMIT - TPA_BREAKPOINT;
			float current_progress = (current_throttle - TPA_BREAKPOINT)
					/ throttle_range;
			tpa_factor = 1.0f - (TPA_RATE * current_progress);
			if (tpa_factor < 0.0f)
				tpa_factor = 0.0f;
		}

		// Inbound TPA Execution
		fc.roll.output = PID_Compute(&fc.roll, fc.target_roll_rate * tpa_factor, angx, dt);
		fc.pitch.output = PID_Compute(&fc.pitch,fc.target_pitch_rate * tpa_factor, angy, dt);
		fc.yaw.output = PID_Compute(&fc.yaw, fc.target_yaw_rate * tpa_factor, angz, dt);

		// --- Step 4: Mixer Output ---
		Mix_Motors_Adjusted(fc.roll.output, fc.pitch.output, fc.yaw.output,
				current_throttle);

	} else {
		// Disarmed Cleanup
		alt_hold_initialized = 0U;
		brake_roll.last_stick_position = 0.0f;  // Flush registers when disarmed
		brake_pitch.last_stick_position = 0.0f;
		PID_Reset(&fc.roll, angx);
		PID_Reset(&fc.pitch, angy);
		PID_Reset(&fc.yaw, angz);
		if (motor_test == false && esc_calibration == false) {
			update_motors(ONESHOT125_MIN, ONESHOT125_MIN, ONESHOT125_MIN,
					ONESHOT125_MIN);
		}
	}

	// Fix 2: Universal Profiling Pin Pull-Down
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
}
// Add this helper function to your file or headers to cleanly isolate the channel math
float normalize_radio_throttle(float raw_throttle) {
	if (raw_throttle < 1000.0f)
		raw_throttle = 1000.0f;
	if (raw_throttle > 2000.0f)
		raw_throttle = 2000.0f;

	// Returns 0.0 at low stick, 0.5 at half stick, and 1.0 at full high stick
	return (float) (raw_throttle - 1000.0f) / 1000.0f;
}
/**
 * @brief Calculates the target angle for Angle Mode, injecting a temporary
 *        counter-angle brake if the pilot quickly snaps the sticks to center.
 */
float Calculate_Braking_Target_Angle(float current_stick, float max_tilt_angle,
		Braking_State_t *state, float dt) {
	if (dt < 0.0001f)
		dt = 0.0001f;

	// 1. Calculate standard pilot target angle based on stick deflection
	float base_target_angle = current_stick * max_tilt_angle;

	// 2. Calculate Stick Velocity (How fast is the pilot moving the stick?)
	float stick_velocity = (current_stick - state->last_stick_position) / dt;
	state->last_stick_position = current_stick; // Update tracking register

	// 3. Detect if the stick is rushing back toward center (0.0f)
	float braking_injection = 0.0f;

	if ((current_stick > 0.05f && stick_velocity < -0.1f)
			|| (current_stick < -0.05f && stick_velocity > 0.1f)) {
		// Inject a counter-force proportional to how violently the stick was centered
		braking_injection = stick_velocity * BRAKING_FORCE;
	}

	// 4. Combine base target angle with the temporary braking modifier
	float final_target_angle = base_target_angle - braking_injection;

	// 5. Hard clamp the output so the braking action never exceeds safe flight envelopes
	float absolute_max_clamp = max_tilt_angle * 1.3f; // Allow 30% overshoot for braking authority
	if (final_target_angle > absolute_max_clamp)
		final_target_angle = absolute_max_clamp;
	if (final_target_angle < -absolute_max_clamp)
		final_target_angle = -absolute_max_clamp;

	return final_target_angle;
}

void get_flight_mode(void) {
	if (radio.mode >= 1000 && radio.mode <= 1300)
		flight_mode = ACRO;
	else if (radio.mode > 1300 && radio.mode <= 1700)
		flight_mode = SELF_LEVEL;
	else if (radio.mode > 1700 && radio.mode <= 2000)
		flight_mode = ALTITUDE_HOLD;
}

