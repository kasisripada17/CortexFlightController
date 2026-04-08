/*
 * lsm6ds3.c
 *
 *  Created on: 07-Apr-2026
 *      Author: kasiviswanadhsripada
 */

#include "lsm6ds3.h"
#include "stm32h7xx_hal.h"

// variables
extern SPI_HandleTypeDef hspi1;
volatile IMU_Data_t raw_sensor_data;
volatile uint8_t imu_data_ready = 0;
volatile uint8_t sensor_data_read = 0;
volatile IMU_Data_t raw_sensor_data;

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

        // 2. Reconstruct 16-bit signed integers (Little Endian)
        raw_sensor_data.gyro_x = (int16_t)((buffer[1] << 8) | buffer[0]);
        raw_sensor_data.gyro_y = (int16_t)((buffer[3] << 8) | buffer[2]);
        raw_sensor_data.gyro_z = (int16_t)((buffer[5] << 8) | buffer[4]);
        raw_sensor_data.acc_x  = (int16_t)((buffer[7] << 8) | buffer[6]);
        raw_sensor_data.acc_y  = (int16_t)((buffer[9] << 8) | buffer[8]);
        raw_sensor_data.acc_z  = (int16_t)((buffer[11] << 8) | buffer[10]);

        double angX = raw_sensor_data.gyro_x * 0.0175f;
        double angY = raw_sensor_data.gyro_y * 0.0175f;
        double angZ = raw_sensor_data.gyro_z * 0.0175f;



        double accX = raw_sensor_data.acc_x * 0.000244f;
        double accY = raw_sensor_data.acc_y * 0.000244f;
        double accZ = raw_sensor_data.acc_z * 0.000244f;

        //
        //		  transmit_size = sprintf((char*)USB_transmit_buffer, "\r\n%f, %f, %f, %f, %f, %f",
        //				 				angX,
        //				 				angY,
        //								angZ,
        //								accX,
        //								accY,
        //								accZ);
        //		 	 CDC_Transmit_HS(USB_transmit_buffer,transmit_size);

    }
}
