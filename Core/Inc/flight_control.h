/*
 * flight_control.h
 *
 *  Created on: 09-Apr-2026
 *      Author: kasiviswanadhsripada
 */

#ifndef INC_FLIGHT_CONTROL_H_
#define INC_FLIGHT_CONTROL_H_





#define MAX_RATE_ACRO 300.0f
#define MAX_TILT_ANGLE 50.0f
// Tune these based on your frame weight: Start very low to prevent slow oscillation
#define ANGLE_KI 0.05f
#define ANGLE_I_MAX 15.0f    // Maximum corrective rate trim cap (degrees/second)
// --- Step 3: TPA Factor Calculation ---
// Example: If your old TPA breakpoint was at 1500 (50% throttle),
// your new breakpoint sits exactly halfway between 34375 and 68750
#define TPA_BREAKPOINT 51562.0f  // 50% Throttle threshold
#define TPA_MAX_LIMIT  68750.0f  // 100% Throttle threshold
#define TPA_RATE       0.35f    // Attenuate P-gains by 35% at 1000% full throttle





typedef enum {
    DISARMED,
    ARMING,
	ARMED_SAFE,
    ARMED,
    DISARMING,
	ESC_CALIBRATION,
	MOTOR_TEST
} arm_state_t;

typedef enum{
	ACRO,
	SELF_LEVEL,
	ALTITUDE_HOLD
}Flight_Mode_t;


float normalize_radio_throttle(float raw_throttle) ;

void flight_control(void) ;
void get_flight_mode(void) ;
float calculate_stick_braking_feedforward(float current_stick, float prev_stick, float dt) ;
#endif /* INC_FLIGHT_CONTROL_H_ */
