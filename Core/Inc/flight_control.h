/*
 * flight_control.h
 *
 *  Created on: 09-Apr-2026
 *      Author: kasiviswanadhsripada
 */

#ifndef INC_FLIGHT_CONTROL_H_
#define INC_FLIGHT_CONTROL_H_



#define MOTOR_MIN 1100  // Minimum spin to keep props moving
#define MOTOR_MAX 2000  // Maximum ESC signal
#define MOTOR_OFF 1000  // Disarmed state


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


void flight_control(void) ;
void get_flight_mode(void) ;
float compute_altitude_hold_throttle(float dt) ;
#endif /* INC_FLIGHT_CONTROL_H_ */
