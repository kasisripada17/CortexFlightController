/*
 * barometer.h
 *
 *  Created on: 05-May-2026
 *      Author: kasiviswanadhsripada
 */
#include <stdint.h>
#include "stm32h7xx_hal.h"


#ifndef INC_BAROMETER_H_
#define INC_BAROMETER_H_


#define MS5611_RESET      0x1E
#define MS5611_PROM_READ  0xA0
#define MS5611_CONV_D1_OSR_4096 0x48 // Pressure (Max precision)
#define MS5611_CONV_D2_OSR_4096 0x58 // Temperature (Max precision)
#define MS5611_ADC_READ   0x00


// Define states for the Barometer
typedef enum {
    BARO_STATE_START_PRES = 0,
    BARO_STATE_WAIT_PRES,
    BARO_STATE_START_TEMP,
    BARO_STATE_WAIT_TEMP,
	BARO_STATE_IDLE
} BaroState_t;







void MS5611_Start_Pressure_Conv(void) ;

void MS5611_Start_Temp_Conv(void) ;
uint32_t MS5611_Read_ADC_Result(void) ;
void Calculate_Final_Altitude(uint32_t D1, uint32_t D2) ;
void run_barometer_state_machine(void) ;





#endif /* INC_BAROMETER_H_ */
