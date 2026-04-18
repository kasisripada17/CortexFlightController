/*
 * flight_control.h
 *
 *  Created on: 09-Apr-2026
 *      Author: kasiviswanadhsripada
 */

#ifndef INC_FLIGHT_CONTROL_H_
#define INC_FLIGHT_CONTROL_H_
void flight_control(void) ;


#define MOTOR_MIN 1100  // Minimum spin to keep props moving
#define MOTOR_MAX 2000  // Maximum ESC signal
#define MOTOR_OFF 1000  // Disarmed state


typedef enum {
    DISARMED,
    ARMING,
    ARMED,
    DISARMING,
	ESC_CALIBRATION,
	GYRO_CALIBRATION
} arm_state_t;

#endif /* INC_FLIGHT_CONTROL_H_ */
