/*
 * sensor_fusion.c
 *
 *  Created on: 18-Apr-2026
 *      Author: kasiviswanadhsripada
 */
#include "gyro_calibration.h"
#include "motion_gc.h"
#include "motion_fx.h"
#include "motion_ac.h"
#include "lsm6ds3.h"
#include <stdio.h>
#include "print.h"
#include "flight_control.h"
#define VERSION_STR_LENG 35
#define SAMPLE_FREQUENCY 416.0f

/* Initialization */
char lib_version[VERSION_STR_LENG];

MGC_knobs_t knobs;
MGC_output_t start_gyro_bias;
float sample_freq;
volatile uint8_t gyro_calib_counter = 0;
volatile uint8_t acc_calib_counter;
extern arm_state_t flight_mode;
extern volatile IMU_Data_t sensor_data;
extern uint8_t buffer[256];
extern uint16_t size;
void gyro_calibration_init(void) {
	sample_freq = SAMPLE_FREQUENCY;
	MotionGC_Initialize(MGC_MCU_STM32, &sample_freq);

	/* Optional: Get version */
	MotionGC_GetLibVersion(lib_version);

	/* Gyroscope calibration API initialization function */

	/* Optional: Get knobs settings */
	MotionGC_GetKnobs(&knobs);
	/* Optional: Adjust knobs settings */
	// Increase Acc threshold to 20mg (Standard is often 0.01 or 0.02)
//	knobs.AccThr = 0.2f;     // Increased from 0.05
//	knobs.GyroThr = 10.0f;    // Increased from 2.0
	// Ensure FastStart is enabled (yours is 1, which is good)
	knobs.FastStart = 1;
	MotionGC_SetKnobs(&knobs);
	/* Optional: Set initial gyroscope offset */
	start_gyro_bias.GyroBiasX = 2.08f;
	start_gyro_bias.GyroBiasY = -7.01f;
	start_gyro_bias.GyroBiasZ = -0.56f;
	MotionGC_SetCalParams(&start_gyro_bias);
	/* Optional: Set sample frequency */
	MotionGC_SetFrequency(&sample_freq);
}

/* Using gyroscope calibration algorithm */
void gyro_calibration_routine() {
	MGC_input_t data_in = {0};
	static MGC_output_t data_out ={0};
	int bias_update= 0;
	data_in.Gyro[0] = sensor_data.gyro_x;
	data_in.Gyro[1] = sensor_data.gyro_y;
	data_in.Gyro[2] = sensor_data.gyro_z;
	data_in.Acc[0] = sensor_data.acc_x;
	data_in.Acc[1] = sensor_data.acc_y;
	data_in.Acc[2] = sensor_data.acc_z;
	if (flight_mode!=ARMED) {


		/* Gyroscope calibration algorithm update */
		MotionGC_Update(&data_in, &data_out, &bias_update);
	}
		/* Apply correction */
		sensor_data.gyro_cal_x = (data_in.Gyro[0] - data_out.GyroBiasX);
		sensor_data.gyro_cal_y = (data_in.Gyro[1] - data_out.GyroBiasY);
		sensor_data.gyro_cal_z = (data_in.Gyro[2] - data_out.GyroBiasZ);
//size  = sprintf(buffer,"\r\n%f,%f,%f",
//		sensor_data.gyro_cal_x,sensor_data.gyro_cal_y,sensor_data.gyro_cal_z);
//usb_print(buffer, size);

}


