/*
 * radio.h
 *
 *  Created on: 07-Apr-2026
 *      Author: kasiviswanadhsripada
 */

#ifndef INC_RADIO_H_
#define INC_RADIO_H_
#include "stm32h7xx_hal.h"

typedef struct{
	float roll;
	float pitch;
	float throttle;
	float yaw;
	float mode;
} receiver_t;

/* User constants for the 50cm aircraft */
#define RC_MIN 1000
#define RC_MAX 2000

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
float normalize_radio(uint16_t pwm_val) ;

#endif /* INC_RADIO_H_ */
