/*
 * barometer.c
 *
 *  Created on: 05-May-2026
 *      Author: kasiviswanadhsripada
 */


#include "barometer.h"
#include <stdio.h>
#include "print.h"
// Global array to store MS5611 calibration coefficients
uint16_t C[7]; // MS5611 has 6 coefficients (1-6). C[0] is typically reserved or CRC.

// Define the PROM read command
#define MS5611_PROM_READ 0xA0
float current_temperature = 0.0f;
float current_pressure = 0.0f;
float current_altitude = 0.0f;
float relative_altitude = 0.0f;
static float P_ground_accumulator = 0; // Temporary sum for averaging
static float P_at_ground = 101325.0f;  // Actual reference pressure
static uint16_t calib_counter = 0;     // Use uint16 for safer counting
#define CALIB_SAMPLES 600  // ~12 seconds of data at 50Hz
#define SKIP_SAMPLES  300  // Skip the first 6 seconds entirely for thermal stability
 // Skip the first 2 seconds to let heat stabilize
extern SPI_HandleTypeDef hspi1;


extern uint8_t buffer[256];

void MS5611_Init(void) {
    // 1. Send Reset Command
    uint8_t reset_cmd = 0x1E;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &reset_cmd, 1, 10);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

    HAL_Delay(10); // Wait for reset to complete

    // 2. Read the PROM (Calibration Coefficients)
    uint8_t buffer[2];
    for (uint8_t i = 0; i < 7; i++) {
        uint8_t prom_cmd = 0xA0 + (i * 2); // 0xA0 to 0xAE

        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, &prom_cmd, 1, 10);
        HAL_SPI_Receive(&hspi1, buffer, 2, 10);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

        C[i] = (buffer[0] << 8) | buffer[1]; // Store in your global C array
    }
}

void MS5611_Start_Pressure_Conv(void) {
    uint8_t cmd = MS5611_CONV_D1_OSR_4096;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET); // D15 LOW
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);   // D15 HIGH
}

void MS5611_Start_Temp_Conv(void) {
    uint8_t cmd = MS5611_CONV_D2_OSR_4096;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET); // D15 LOW
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);   // D15 HIGH
}
uint32_t MS5611_Read_ADC_Result(void) {
    uint8_t cmd = MS5611_ADC_READ;
    uint8_t buffer[3];

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET); // D15 LOW
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
    HAL_SPI_Receive(&hspi1, buffer, 3, 10);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);   // D15 HIGH

    // Combine the three 8-bit registers into one 24-bit value
    return ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
}

void Calculate_Final_Altitude(uint32_t D1, uint32_t D2) {
	// 1. Calculate temperature difference (dT)
	int64_t dT = (int64_t) D2 - ((int64_t) C[5] << 8);

	// 2. Calculate actual temperature (initial)
	int32_t TEMP = 2000 + ((dT * C[6]) >> 23);

	// 3. SECOND ORDER TEMPERATURE COMPENSATION
	// Corrects for non-linearities at low/high temperatures
	int64_t T2 = 0;
	int64_t OFF2 = 0;
	int64_t SENS2 = 0;

	if (TEMP < 2000) { // If Temperature < 20°C
		T2 = (dT * dT) >> 31;
		OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) >> 1;
		SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) >> 2;

		if (TEMP < -1500) { // If Temperature < -15°C
			OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
			SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) >> 1;
		}
	}

	TEMP = TEMP - T2; // Final compensated temperature

	// 4. Calculate pressure offset (OFF) and sensitivity (SENS)
	int64_t OFF = ((int64_t) C[2] << 16) + (((int64_t) C[4] * dT) >> 7);
	int64_t SENS = ((int64_t) C[1] << 15) + (((int64_t) C[3] * dT) >> 8);

	OFF = OFF - OFF2;
	SENS = SENS - SENS2;

	// 5. Calculate temperature-compensated pressure (P)
	int32_t P = (int32_t) (((((int64_t) D1 * SENS) >> 21) - OFF) >> 15);

	// 6. Convert to Floats for ease of use
	current_temperature = (float) TEMP / 100.0f; // Celsius
	current_pressure = (float) P;               // Pascals

	// 7. Calculate Altitude relative to takeoff point
	// Use P_at_ground (the pressure captured when the drone was armed)
	current_altitude = 44330.0f
			* (1.0f - powf(current_pressure / P_at_ground, 0.190295f));


	// 7. BOOT CALIBRATION LOGIC
	if (calib_counter < CALIB_SAMPLES) {
		// Skip the first few samples to let the sensor settle (optional but recommended)
		if (calib_counter > SKIP_SAMPLES) {
		            P_ground_accumulator += current_pressure;
		        }
		calib_counter++;

		// While calibrating, we don't have a valid relative altitude yet
		relative_altitude = 0.0f;
	} else if (calib_counter == CALIB_SAMPLES) {
		// Calculate the final average ground pressure
		// (Adjusted for the 20 skipped samples)
		P_at_ground = P_ground_accumulator / (float)(CALIB_SAMPLES - SKIP_SAMPLES);
		calib_counter++;
	} else {
		// 8. NORMAL FLIGHT CALCULATION
		// Calculate altitude relative to the captured ground pressure
		relative_altitude = 44330.0f
				* (1.0f - powf(current_pressure / P_at_ground, 0.190295f));
		uint8_t size = sprintf(buffer, "%f\r\n", relative_altitude);
		usb_print(buffer, size);

	}


}



