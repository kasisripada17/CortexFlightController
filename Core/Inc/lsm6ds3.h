/*
 * lsm6ds3.h
 *
 *  Created on: 07-Apr-2026
 *      Author: kasiviswanadhsripada
 */

#ifndef INC_LSM6DS3_H_
#define INC_LSM6DS3_H_
#include <stdint.h>

#define LOOP_FREQ (float)1666.0f
#define DT (float)(1.0f/LOOP_FREQ)


#define LSM6DS3_ADDR_WHO_AM_I  0x0F
#define LSM6DS3_WHO_AM_I_VAL   0x69
#define CS_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_4
#define GYRO_MAX_SAMPLES 2000
#define ACC_MAX_SAMPLES 2000

typedef struct {
    float gyro_x, gyro_y, gyro_z;
    float gyro_cal_x, gyro_cal_y, gyro_cal_z;
    float acc_x, acc_y, acc_z;
    float acc_cal_x, acc_cal_y, acc_cal_z;
    float roll,pitch,yaw;
    float acc_earth_x,acc_earth_y,acc_earth_z;
    float fused_alt;
    float fused_vel;
} IMU_Data_t;

typedef struct {
    float gx_offset;
    float gy_offset;
    float gz_offset;
    float ax_offset;
    float ay_offset;
    float az_offset;
} IMU_Config_t;



typedef enum {
	NOT_STARTED = 0,
	CALIBRATING,
	CALIBRATED
} Sensor_Calibration;



 // Function Prototypes
 void LSM6DS3_Calibrate(uint16_t num_samples);
void IMU_Write_Reg(uint8_t reg, uint8_t value) ;
uint8_t IMU_Read_Reg(uint8_t reg_addr) ;
uint8_t IMU_Init(void) ;



#endif /* INC_LSM6DS3_H_ */
