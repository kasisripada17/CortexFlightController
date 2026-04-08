/*
 * lsm6ds3.h
 *
 *  Created on: 07-Apr-2026
 *      Author: kasiviswanadhsripada
 */

#ifndef INC_LSM6DS3_H_
#define INC_LSM6DS3_H_
#include <stdint.h>

typedef struct {
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t acc_x, acc_y, acc_z;
} IMU_Data_t;


#define LSM6DS3_ADDR_WHO_AM_I  0x0F
#define LSM6DS3_WHO_AM_I_VAL   0x69
#define CS_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_4

void IMU_Write_Reg(uint8_t reg, uint8_t value) ;
uint8_t IMU_Read_Reg(uint8_t reg_addr) ;
uint8_t IMU_Init(void) ;



#endif /* INC_LSM6DS3_H_ */
