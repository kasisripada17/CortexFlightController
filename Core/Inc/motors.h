/*
 * motors.h
 *
 *  Created on: 07-Apr-2026
 *      Author: kasiviswanadhsripada
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_


#include <stdint.h>
#include "stm32h7xx_hal.h"
// Mathematical conversion boundaries for 275 MHz clock execution:
// 125 microseconds * 275 ticks/us = 34,375 (Zero Throttle pulse width)
// 250 microseconds * 275 ticks/us = 68,750 (Full Throttle pulse width)
#define ONESHOT125_MIN  34375.0f
#define ONESHOT125_MAX  68750.0f
#define MOTOR_IDLE_GAP  6875.0f    // ~5% idle threshold for reliable motor spin-up
#define MOTOR_ARMED_IDLE (ONESHOT125_MIN + MOTOR_IDLE_GAP) // 36,093 counts
#define ONE_SHOT_ESCS
//#define TRADITIONAL_PWM_ESCS
#define PWM_MIN 1000.0f
#define PWM_MAX 2000.0f
#define PWM_IDLE 1100.0f

void update_motors(uint32_t m1, uint32_t m2, uint32_t m3, uint32_t m4) ;


#endif /* INC_MOTORS_H_ */
