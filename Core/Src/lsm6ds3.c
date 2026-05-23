/*
 * lsm6ds3.c
 *
 *  Created on: 07-Apr-2026
 *      Author: kasiviswanadhsripada
 */
#include <stdbool.h>
#include "gyro_calibration.h"
#include "lsm6ds3.h"
#include "stm32h7xx_hal.h"
#include "print.h"
#include "radio.h"
#include "pid_control.h"
#include "motors.h"
#include "flight_control.h"
#include "sensor_fusion.h"
#include "barometer.h"

extern float relative_altitude;
extern float a_global[3];
extern float alt_fused;

// variables
extern SPI_HandleTypeDef hspi1;
volatile IMU_Data_t sensor_data = { 0 };
volatile uint8_t imu_data_ready = 0;
volatile uint8_t sensor_data_read = 0;
extern uint32_t receiver[4];
extern Flight_Control_t fc;
extern volatile receiver_t radio;
IMU_Config_t imu_offsets = { 0 };
uint16_t sample_number = 0;
Sensor_Calibration gyro_calibration = NOT_STARTED;
uint32_t motor[4];
extern bool start_gyro_calibration;
volatile uint32_t gyro_calib_counter = 0;
volatile uint32_t acc_calib_counter = 0;
extern TIM_HandleTypeDef htim2;
extern uint8_t buffer[256];
extern uint16_t size;

BaroState_t currentBaroState = BARO_STATE_IDLE;
uint32_t lastBaroTime = 0;
// Raw data variables for the MS5611
uint32_t D1 = 0; // Raw Pressure
uint32_t D2 = 0; // Raw Temperature

// Ensure the calibration array is also present
extern uint16_t C[7];
//functions

void IMU_Write_Reg(uint8_t reg, uint8_t value) {
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &reg, 1, 10);
	HAL_SPI_Transmit(&hspi1, &value, 1, 10);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

uint8_t IMU_Read_Reg(uint8_t reg_addr) {
	uint8_t command = reg_addr | 0x80; // Set MSB to 1 for Read operation
	uint8_t read_val = 0;

	// 1. Pull CS Low to select the LSM6DS3
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	// 2. Send the register address
	HAL_SPI_Transmit(&hspi1, &command, 1, 10);

	// 3. Receive the register data
	HAL_SPI_Receive(&hspi1, &read_val, 1, 10);

	// 4. Pull CS High to end the transaction
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	return read_val;
}
uint8_t IMU_Init(void) {



	MS5611_Init();


	if (IMU_Read_Reg(0x0F) != 0x69) {
		return 0;
	}


	    IMU_Write_Reg(0x12, 0x05); // SW Reset
	    HAL_Delay(100);            // Give it more time to stabilize

	    // 1. SET ODR FIRST (6.66 kHz)
	    IMU_Write_Reg(0x10, 0x8E); // Accel: 6.66kHz, 8g

	    IMU_Write_Reg(0x11, 0x84); // Gyro: 6.66kHz, 500dps

	    // 2. DISABLE ALL LOW POWER MODES (Crucial)
	    // CTRL6_C (0x15): Bit 4 = 0 (XL_HM_MODE = High Perf)
	    // Also, Bits 0-2 (FTYPE) should be 000 for max bandwidth.
	    IMU_Write_Reg(0x15, 0x00);

	    // CTRL7_G (0x16): Bit 7 = 0 (G_HM_MODE = High Perf)
	    // Bit 2 = 1 (Rounding disabled / High Performance force)
	    // IMPORTANT: Set this to 0x00 or 0x04.
	    IMU_Write_Reg(0x16, 0x00);

	    // 3. ENABLE BLOCK DATA UPDATE
	    IMU_Write_Reg(0x12, 0x44);
	    IMU_Write_Reg(0x13, 0x00);

	    // 4. INTERRUPT CONFIG
	    // Switch INT1_CTRL back to 0x02 (Gyro DRDY).
	    // Now that HM_MODE is forced, it should pulse at 6.66kHz.
	    IMU_Write_Reg(0x0D, 0x02);
	return 1;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == GPIO_PIN_4) { // PC4 triggered

//		static uint32_t now = 0;
//		static uint32_t prev = 0;
//
//		now = __HAL_TIM_GET_COUNTER(&htim2);
//		size = sprintf(buffer, "\r\n%d", now - prev);
//		usb_print(buffer, size);
//		prev = now;

// 1. Start SPI Burst Read (12 bytes)
		// Address 0x22 (GyroX_L) | 0x80 (Read Bit)
		uint8_t reg = 0x22 | 0x80;
		uint8_t buffer[12];

		// Manual CS Low
		CS_GPIO_Port->BSRR = (uint32_t) CS_Pin << 16;

		// Send address and receive 12 bytes
		HAL_SPI_Transmit(&hspi1, &reg, 1, 10);
		HAL_SPI_Receive(&hspi1, buffer, 12, 10);

		// Manual CS High
		CS_GPIO_Port->BSRR = CS_Pin;

		//calibration routine
		int16_t angx = 0, angy = 0, angz = 0;
		int16_t accx = 0, accy = 0, accz = 0;
		// 2. Reconstruct 16-bit signed integers (Little Endian)
		angx = (int16_t) ((buffer[1] << 8) | buffer[0]);
		angy = (int16_t) ((buffer[3] << 8) | buffer[2]);
		angz = (int16_t) ((buffer[5] << 8) | buffer[4]);
		accx = (int16_t) ((buffer[7] << 8) | buffer[6]);
		accy = (int16_t) ((buffer[9] << 8) | buffer[8]);
		accz = (int16_t) ((buffer[11] << 8) | buffer[10]);




		sensor_data.gyro_x = ((float) (angx * 0.0175f));
		sensor_data.gyro_y = ((float) (angy * 0.0175f));
		sensor_data.gyro_z = ((float) (angz * 0.0175f));

		// 1. Subtract offsets and scale to G's (+/- 8g scale)
		sensor_data.acc_x = ((float) (accx * 0.000244f));
		sensor_data.acc_y = ((float) (accy * 0.000244f));
		sensor_data.acc_z = ((float) (accz * 0.000244f));


		gyro_calibration_routine();


		static float acc_bias[3];
		static float gyro_bias[3];

		static bool acc_calib_done = false;
		static float prevx = 0.0f, prevy = 0.0f, prevz = 0.0f;
#define CALIB_SAMPLES (float)1000.0f
		if (!acc_calib_done) {
			if (acc_calib_counter < (int)CALIB_SAMPLES) {

				if (fabsf(sensor_data.acc_x - prevx) > 0.1f
						|| fabsf(sensor_data.acc_y - prevy) > 0.1f) {

					acc_bias[0] = 0.0f;
					acc_bias[1] = 0.0f;
					acc_bias[2] = 0.0f;
					gyro_bias[0] = 0.0f;
					gyro_bias[1] = 0.0f;
					gyro_bias[2] = 0.0f;

					acc_calib_counter = 0;
				} else {
					acc_bias[0] += sensor_data.acc_x;
					acc_bias[1] += sensor_data.acc_y;
					acc_bias[2] += sensor_data.acc_z; // Accumulate Z
					gyro_bias[0] += sensor_data.gyro_x;
					gyro_bias[1] += sensor_data.gyro_y;
					gyro_bias[2] += sensor_data.gyro_z; // Accumulate Z
					acc_calib_counter++;
				}
			} else {
				acc_bias[0] /= CALIB_SAMPLES;
				acc_bias[1] /= CALIB_SAMPLES;
				acc_bias[2] = (acc_bias[2] /CALIB_SAMPLES) - 1.0f;
				gyro_bias[0] /= CALIB_SAMPLES;
				gyro_bias[1] /= CALIB_SAMPLES;
				gyro_bias[2] /= CALIB_SAMPLES;

				acc_calib_done = true;
			}
		}
		if (acc_calib_done) {
			sensor_data.acc_x = ((float) (sensor_data.acc_x - acc_bias[0]));
			sensor_data.acc_y = ((float) (sensor_data.acc_y - acc_bias[1]));
			sensor_data.acc_z = ((float) (sensor_data.acc_z - acc_bias[2]));
//			sensor_data.gyro_cal_x =  ((float) (sensor_data.gyro_x - gyro_bias[0]));
//			sensor_data.gyro_cal_y =  ((float) (sensor_data.gyro_y - gyro_bias[1]));
//			sensor_data.gyro_cal_z =  ((float) (sensor_data.gyro_z - gyro_bias[2]));

		}
		prevx = sensor_data.acc_x;
		prevy = sensor_data.acc_y;
		prevz = sensor_data.acc_z;


		motion_fx_update();

		flight_control();

		// 2. RUN BARO STATE MACHINE (Non-blocking)
		switch (currentBaroState) {
		case BARO_STATE_IDLE:
			MS5611_Start_Pressure_Conv(); // Send command, pull D15 HIGH
			lastBaroTime = HAL_GetTick();
			currentBaroState = BARO_STATE_WAIT_PRES;
			break;

		case BARO_STATE_WAIT_PRES:
			if (HAL_GetTick() - lastBaroTime >= 10) { // Check if 10ms passed
				D1 = MS5611_Read_ADC_Result();    // Pull D15 LOW, read, HIGH
				MS5611_Start_Temp_Conv();
				lastBaroTime = HAL_GetTick();
				currentBaroState = BARO_STATE_WAIT_TEMP;
			}
			break;

		case BARO_STATE_WAIT_TEMP:
			if (HAL_GetTick() - lastBaroTime >= 10) {
				D2 = MS5611_Read_ADC_Result();
				Calculate_Final_Altitude(D1, D2);
				float dt = DT;
				alt_fused = update_altitude_fusion(relative_altitude,
						a_global[2], dt);
				//update_tuning_from_radio();

				currentBaroState = BARO_STATE_IDLE; // Start over
			}
			break;
		}

	}

}

