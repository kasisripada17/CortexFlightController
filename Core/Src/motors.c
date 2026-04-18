/*
 * motors.c
 *
 *  Created on: 07-Apr-2026
 *      Author: kasiviswanadhsripada
 */

#include "motors.h"
#include <stdio.h>
#include "print.h"
#include "flight_control.h"
extern TIM_HandleTypeDef htim2;

extern 	uint8_t buffer[256];
extern uint16_t size;
extern arm_state_t flight_mode ;

void update_motors(uint32_t m1, uint32_t m2, uint32_t m3, uint32_t m4) {
if(flight_mode == ARMED)
{
		size = sprintf(buffer, "m1=%d, m2=%d, m3=%d, m4=%d\r\n",
				m1,m2,m3,m4);
}
		usb_print(buffer, size);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, m1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, m2);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, m3);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, m4);
}

