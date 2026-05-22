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
	float pid_channel_selector;
	float pid_tune_gain_selector;
	float p_gain;
	float i_gain;
	float d_gain;

} receiver_t;

/* User constants for the 50cm aircraft */
#define RC_MIN 1000
#define RC_MAX 2000
//subs receiver section

#define SBUS_RECEIVER
//#define TRADITIONAL_RECEIVER


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
float normalize_radio(uint16_t pwm_val) ;
void Parse_SBUS() ;
void Update_PWM_Targets() ;
uint16_t Map_SBUS_to_PWM(uint16_t sbus_val) ;
void update_tuning_from_radio(void) ;
#endif /* INC_RADIO_H_ */
