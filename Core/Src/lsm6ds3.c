/*
 * lsm6ds3.c
 *
 *  Created on: 07-Apr-2026
 *      Author: kasiviswanadhsripada
 */

#include <gyro_calibration.h>
#include "lsm6ds3.h"
#include "stm32h7xx_hal.h"
#include "print.h"
#include "radio.h"
#include "pid_control.h"
#include "motors.h"
#include "flight_control.h"
#include <stdbool.h>
#include "sensor_fusion.h"
// variables
extern SPI_HandleTypeDef hspi1;
volatile IMU_Data_t sensor_data = { 0 };
volatile uint8_t imu_data_ready = 0;
volatile uint8_t sensor_data_read = 0;
extern uint32_t receiver[4];
extern Flight_Control_t fc;
extern receiver_t radio;
IMU_Config_t imu_offsets = { 0 };
uint16_t sample_number = 0;
Sensor_Calibration gyro_calibration = NOT_STARTED;
uint32_t motor[4];
extern bool start_gyro_calibration;
 volatile uint32_t gyro_calib_counter = 0;
 volatile uint32_t acc_calib_counter = 0;

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
	uint8_t whoAmI = 0;

	// 1. Check Communication
	uint8_t reg = LSM6DS3_ADDR_WHO_AM_I | 0x80; // Read bit
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &reg, 1, 10);
	HAL_SPI_Receive(&hspi1, &whoAmI, 1, 10);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	// 1. Verify Communication (Expects 0x69)
	if (IMU_Read_Reg(0x0F) != 0x69) {
		while (1) {
		}
		return 0;
	}

	// 1. Reset the device to a known clean state
	IMU_Write_Reg(0x12, 0x05); // SW_RESET=1, IF_INC=1
	HAL_Delay(50);             // Increased delay to ensure full reboot

	// 2. Configure Full Scale Ranges (8g and 500dps)
	// Note: We leave ODR at 0 for now during configuration
	IMU_Write_Reg(0x10, 0x0C); // 8g range
	IMU_Write_Reg(0x11, 0x04); // 500dps range

	// 3. System & Interface Config
	// BDU=1 prevents reading "split" data; IF_INC=1 allows burst reads
	IMU_Write_Reg(0x12, 0x44);

	// 4. Hardware Filtering Setup (The "Anti-Vibration" Engine)
	// Enable LPF2 for Accel and set BW to ODR/9 (~46Hz)
	IMU_Write_Reg(0x17, 0x89);

	// Enable LPF2 for Gyroscope
	//IMU_Write_Reg(0x13, 0x02);

	// Set Gyro LPF2 to strongest filtering (FTYPE=11)
	IMU_Write_Reg(0x15, 0x03);

	// 5. Final Step: Enable Data Flow by setting ODR to 416Hz
	// Doing this last ensures the filter pipeline is ready as data starts flowing
	IMU_Write_Reg(0x10, 0x6C); // 416Hz + 8g
	IMU_Write_Reg(0x11, 0x64); // 416Hz + 500dps

	// 6. Sync & Interrupts
	IMU_Write_Reg(0x0D, 0x02); // Route Gyro Data Ready to INT1

	// 7. CRITICAL: Settling Delay
	// This prevents your arming logic from seeing "zeroed out" or
	// unstable filtered data during the first few cycles.
	HAL_Delay(100);

	return 1; // Success
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_4) { // PC4 triggered

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



		static float acc_bias[3];
		static bool acc_calib_done = false;
		if (!acc_calib_done) {
		    if (acc_calib_counter < 1000) {
		        acc_bias[0] += sensor_data.acc_x;
		        acc_bias[1] += sensor_data.acc_y;
		        acc_calib_counter++;
		    }
		    else {
		        acc_bias[0] /= 1000.0f;
		        acc_bias[1] /= 1000.0f;
		        acc_calib_done = true;
		    }
		}
		if (acc_calib_done) {
			sensor_data.acc_x = ((float) (sensor_data.acc_x - acc_bias[0]));
			sensor_data.acc_y = ((float) (sensor_data.acc_y - acc_bias[1]));
		}

		gyro_calibration_routine();
		motion_fx_update();

		flight_control();

	}

}

