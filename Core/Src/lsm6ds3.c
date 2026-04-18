/*
 * lsm6ds3.c
 *
 *  Created on: 07-Apr-2026
 *      Author: kasiviswanadhsripada
 */

#include "lsm6ds3.h"
#include "stm32h7xx_hal.h"
#include "print.h"
#include "radio.h"
#include "pid_control.h"
#include "motors.h"
#include "flight_control.h"
#include <stdbool.h>
// variables
extern SPI_HandleTypeDef hspi1;
volatile IMU_Data_t sensor_data;
volatile uint8_t imu_data_ready = 0;
volatile uint8_t sensor_data_read = 0;
extern uint32_t receiver[4];
extern Flight_Control_t fc ;
extern receiver_t radio;
IMU_Config_t imu_offsets = {0};
uint16_t sample_number = 0;
Sensor_Calibration gyro_calibration = NOT_STARTED;
uint32_t motor[4];
extern bool start_gyro_calibration;
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
	if (IMU_Read_Reg(0x0F) != 0x69)
	{
		while(1)
		{
		}
		return 0;
	}

    // 2. Software Reset (Recommended for clean state)
    IMU_Write_Reg(0x12, 0x05); // SW_RESET=1, IF_INC=1 (Auto-increment for burst reads)
    HAL_Delay(10);             // Wait for reboot

    // 3. Accelerometer Config: 1.66 kHz ODR, 8g Full Scale
    IMU_Write_Reg(0x10, 0x8C); // [1000 1100]

    // 4. Gyroscope Config: 1.66 kHz ODR, 500 dps Full Scale
    IMU_Write_Reg(0x11, 0x84); // [1000 0100]

    // 5. Hardware Filtering (Crucial for Hard-Mount noise)
    IMU_Write_Reg(0x13, 0x80); // Enable LPF2 (Secondary digital low-pass filter)
    IMU_Write_Reg(0x17, 0x00); // Set Accel LPF2 cutoff to ODR/50 (approx 33Hz)

    // 6. Interrupt Mapping (Syncs your 1.66 kHz PID loop)
    IMU_Write_Reg(0x0D, 0x02); // Route Gyro Data Ready to INT1 pin (PC4)

    // 7. Global Config
    IMU_Write_Reg(0x12, 0x44); // BDU=1 (Block Data Update) + IF_INC=1

    return 1; // Success
}




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_4) { // PC4 triggered

        // 1. Start SPI Burst Read (12 bytes)
        // Address 0x22 (GyroX_L) | 0x80 (Read Bit)
        uint8_t reg = 0x22 | 0x80;
        uint8_t buffer[12];

        // Manual CS Low
        CS_GPIO_Port->BSRR = (uint32_t)CS_Pin << 16;

        // Send address and receive 12 bytes
        HAL_SPI_Transmit(&hspi1, &reg, 1, 10);
        HAL_SPI_Receive(&hspi1, buffer, 12, 10);

        // Manual CS High
        CS_GPIO_Port->BSRR = CS_Pin;

        //calibration routine



        // 2. Reconstruct 16-bit signed integers (Little Endian)
        sensor_data.raw_gyrox = (int16_t)((buffer[1] << 8) | buffer[0]);
        sensor_data.raw_gyroy = (int16_t)((buffer[3] << 8) | buffer[2]);
        sensor_data.raw_gyroz = (int16_t)((buffer[5] << 8) | buffer[4]);
        sensor_data.raw_accx  = (int16_t)((buffer[7] << 8) | buffer[6]);
        sensor_data.raw_accy  = (int16_t)((buffer[9] << 8) | buffer[8]);
        sensor_data.raw_accz  = (int16_t)((buffer[11] << 8) | buffer[10]);





        if (start_gyro_calibration && gyro_calibration != CALIBRATED) {
			if (HAL_GetTick() > 3000) {
				if (sample_number < GYRO_MAX_SAMPLES) {
					// gyro accumulation
					imu_offsets.gx_offset += (float) sensor_data.raw_gyrox;
					imu_offsets.gy_offset += (float) sensor_data.raw_gyroy;
					imu_offsets.gz_offset += (float) sensor_data.raw_gyroz;

					// accel accumulation
					imu_offsets.ax_offset += (float) sensor_data.raw_accx;
					imu_offsets.ay_offset += (float) sensor_data.raw_accy;
					imu_offsets.az_offset += (float) sensor_data.raw_accz;
					sample_number++;

				} else if (sample_number == GYRO_MAX_SAMPLES) {
					imu_offsets.gx_offset /= (float) GYRO_MAX_SAMPLES;
					imu_offsets.gy_offset /= (float) GYRO_MAX_SAMPLES;
					imu_offsets.gz_offset /= (float) GYRO_MAX_SAMPLES;

					// Finalize Accel
					imu_offsets.ax_offset /= (float) ACC_MAX_SAMPLES;
					imu_offsets.ay_offset /= (float) ACC_MAX_SAMPLES;

					/* Z-Axis Logic:
					 Assuming +/- 8g scale, 1g = 4096 LSB.
					 We subtract 4096 because we want the offset to represent
					 the ERROR away from 1g, not the gravity itself.
					 */
					imu_offsets.az_offset = (imu_offsets.az_offset
							/ (float) ACC_MAX_SAMPLES) - 4096.0f;

					gyro_calibration = CALIBRATED;
					start_gyro_calibration = false;
				}
			}

		}
        else if (gyro_calibration == CALIBRATED)
        {

			sensor_data.gyro_x = ((float)sensor_data.raw_gyrox - imu_offsets.gx_offset) * 0.0175f;
			sensor_data.gyro_y  = ((float)sensor_data.raw_gyroy - imu_offsets.gy_offset) * 0.0175f;
			sensor_data.gyro_z= ((float)sensor_data.raw_gyroz - imu_offsets.gz_offset) * 0.0175f;

			// 1. Subtract offsets and scale to G's (+/- 8g scale)
			sensor_data.acc_x = ((float)sensor_data.raw_accx - imu_offsets.ax_offset) * 0.000244f;
			sensor_data.acc_y = ((float)sensor_data.raw_accy - imu_offsets.ay_offset) * 0.000244f;
			sensor_data.acc_z = ((float)sensor_data.raw_accz - imu_offsets.az_offset) * 0.000244f;

			// 2. Calculate Pitch and Roll angles in degrees
			    // atan2 returns radians, so we multiply by 57.2958 (180/PI)
			//float accel_pitch = atan2f(-sensor_data.acc_x, sqrtf(sensor_data.acc_y * sensor_data.acc_y + sensor_data.acc_z * sensor_data.acc_z)) * 57.2958f;
			//float accel_roll  = atan2f(sensor_data.acc_y, sensor_data.acc_z) * 57.2958f;

			flight_control();


        }






    }
}




