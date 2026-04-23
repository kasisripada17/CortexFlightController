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
volatile IMU_Data_t sensor_data = {0};
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
extern volatile uint8_t gyro_calib_counter;
extern volatile uint8_t acc_calib_counter;

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

	// 2. Software Reset (Recommended for clean state)
	IMU_Write_Reg(0x12, 0x05); // SW_RESET=1, IF_INC=1 (Auto-increment for burst reads)
	HAL_Delay(10);             // Wait for reboot




	// Accelerometer: 416 Hz, 8g
	IMU_Write_Reg(0x10, 0x6C);

	// Gyroscope: 416 Hz, 500 dps
	IMU_Write_Reg(0x11, 0x64);

	// CTRL3_C: BDU=1, IF_INC=1 -> 0x44 (Ensure bits match your SPI wiring)
	IMU_Write_Reg(0x12, 0x44);

	// CTRL8_XL: Enable LPF2 for Accel (Low Pass Filter)
	IMU_Write_Reg(0x17, 0x80);









	// 5. Hardware Filtering (Crucial for Hard-Mount noise)
	IMU_Write_Reg(0x13, 0x80); // Enable LPF2 (Secondary digital low-pass filter)

	// 6. Interrupt Mapping (Syncs your 1.66 kHz PID loop)
	IMU_Write_Reg(0x0D, 0x02); // Route Gyro Data Ready to INT1 pin (PC4)


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
		int16_t angx=0,angy=0,angz = 0;
		int16_t accx=0,accy=0,accz = 0;
		// 2. Reconstruct 16-bit signed integers (Little Endian)
		angx = (int16_t) ((buffer[1] << 8) | buffer[0]);
		angy = (int16_t) ((buffer[3] << 8) | buffer[2]);
		angz = (int16_t) ((buffer[5] << 8) | buffer[4]);
		accx = (int16_t) ((buffer[7] << 8) | buffer[6]);
		accy = (int16_t) ((buffer[9] << 8) | buffer[8]);
		accz = (int16_t) ((buffer[11] << 8) | buffer[10]);

		gyro_calib_counter++;
		acc_calib_counter++;

		sensor_data.gyro_x = ((float) (angx * 0.0175f));
		sensor_data.gyro_y = ((float) (angy * 0.0175f));
		sensor_data.gyro_z = ((float) (angz * 0.0175f));

		// 1. Subtract offsets and scale to G's (+/- 8g scale)
		sensor_data.acc_x = ((float) accy * 0.000244f);
		sensor_data.acc_y = ((float) accx * 0.000244f);
		sensor_data.acc_z = ((float) accz * 0.000244f);
		gyro_calibration_routine();
		 motion_fx_update();

		 flight_control();

	}

}

