/*
 * motion_FX.c
 *
 *  Created on: 19-Apr-2026
 *      Author: kasiviswanadhsripada
 */
#include "motion_fx.h"
#include "stm32h7xx_hal.h"
#include "lsm6ds3.h"
#include "print.h"
#define MFX_STR_LENG 35
#define STATE_SIZE (uint32_t)(2450)
#define ENABLE_6X 1
char lib_version_mfx[MFX_STR_LENG];
static uint8_t mfxstate[STATE_SIZE ];
MFX_knobs_t iKnobs;
float LastTime;
extern volatile IMU_Data_t sensor_data;
extern uint8_t buffer[256];
//	/* USER CODE END 2 */
	 float roll_copter = 0.0f;
	 float pitch_copter = 0.0f;
void motionfx_init(void) {
	/* Check if statically allocated memory size is sufficient
	 to store MotionFX algorithm state and resize if necessary */
	if (STATE_SIZE < MotionFX_GetStateSize()) {
		__disable_irq();
		while (1) {
		}
	}

	/* Sensor Fusion API initialization function */
	MotionFX_initialize((MFXState_t*) mfxstate);
	/* Optional: Get version */
	MotionFX_GetLibVersion(lib_version_mfx);
	/* Modify knobs settings & set the knobs */
	MotionFX_getKnobs(mfxstate, &iKnobs);
	// 2. Adjust for a Drone (High vibration environment)
	iKnobs.ATime = 10.0f;  // Increase this if the horizon "shakes" during flight
	iKnobs.FrTime = 2.0f;        // Snap to heading quickly
	iKnobs.LMode = 0;           // High-dynamic mode for aircraft
	iKnobs.modx = 1;            // Enable Gyro Bias estimation within the EKF
	iKnobs.acc_orientation[0] ='w';
	iKnobs.acc_orientation[1] ='s';
	iKnobs.acc_orientation[2] ='u';

	iKnobs.gyro_orientation[0] = 'w';
	iKnobs.gyro_orientation[1] = 's';
	iKnobs.gyro_orientation[2] = 'u';

	// 3. Set the coordinate system
	// Most flight controllers use NED (North East Down)
	iKnobs.output_type = MFX_ENGINE_OUTPUT_NED;
	MotionFX_setKnobs(mfxstate, &iKnobs);
	MotionFX_enable_6X(mfxstate, MFX_ENGINE_DISABLE);
	MotionFX_enable_9X(mfxstate, MFX_ENGINE_DISABLE);
	/* Enable 9-axis sensor fusion */
	if (ENABLE_6X == 1) {
		MotionFX_enable_6X(mfxstate, MFX_ENGINE_ENABLE);
	} else {
		MotionFX_enable_9X(mfxstate, MFX_ENGINE_ENABLE);
	}
}

/* Using Sensor Fusion algorithm */
void motion_fx_update(void) {
	MFX_input_t data_in;
	MFX_output_t data_out;
	float dT = 0.002403846f;
	static float gyrox = 0.0f, gyroy = 0.0f, gyroz = 0.0f;
	static float accx = 0.0f, accy = 0.0f, accz = 0.0f;

	gyrox = gyrox * 0.5f + (sensor_data.gyro_cal_x) * 0.5f;
	gyroy = gyroy * 0.5f + (sensor_data.gyro_cal_y) * 0.5f;
	gyroz = gyroz * 0.5f + (sensor_data.gyro_cal_z) * 0.5f;


	accx = accx * 0.8f + (sensor_data.acc_x) * 0.2f;
	accy = accy * 0.8f + (sensor_data.acc_y) * 0.2f;
	accz = accz * 0.8f + (sensor_data.acc_z) * 0.2f;


//	data_in.gyro[0] = gyrox;
//	data_in.gyro[1] = gyroy;
//	data_in.gyro[2] = gyroz;
//
//	data_in.acc[0] = accx;
//	data_in.acc[1] = accy;
//	data_in.acc[2] = accz;

	data_in.acc[0] = sensor_data.acc_x;
	data_in.acc[1] = sensor_data.acc_y;
	data_in.acc[2] = sensor_data.acc_z;

	data_in.gyro[0] = sensor_data.gyro_cal_x;
	data_in.gyro[1] = sensor_data.gyro_cal_y;
	data_in.gyro[2] = sensor_data.gyro_cal_z;

//	float roll_acc = 0.0f;
//	float pitch_acc = 0.0f;
//   1. Calculate Total G-Force
//	float resultant_acceleration = sqrt(
//	    (sensor_data.acc_x * sensor_data.acc_x) +
//	    (sensor_data.acc_y * sensor_data.acc_y) +
//	    (sensor_data.acc_z * sensor_data.acc_z)
//	);
//
//	// 2. Calculate Accelerometer Angles (Trigonometry)
//	if (resultant_acceleration > 0.5f) { // Ensure we aren't in freefall
//	    if (fabs(sensor_data.acc_y) < resultant_acceleration) {
//	        pitch_acc = -asin(sensor_data.acc_y / resultant_acceleration) * 57.296f;
//	    }
//	    if (fabs(sensor_data.acc_x) < resultant_acceleration) {
//	        // Negated to match your "Right is Negative" convention
//	        roll_acc = -asin(sensor_data.acc_x / resultant_acceleration) * -57.296f;
//	    }
//	}
//
//	// 3. Gyro Integration (The "Fast" update)
//	// Note: Pitch uses Gyro Y, Roll uses Gyro X
//	pitch_copter += (sensor_data.gyro_cal_y) * dT;
//	roll_copter  += (sensor_data.gyro_cal_x) * dT;
//
//	// 4. The Complementary Filter (The "Trust Factor")
//	// This pulls the gyro drift back to the accelerometer's gravity vector
//	roll_copter  = 0.999f * roll_copter  + 0.001f * roll_acc;
//	pitch_copter = 0.999f * pitch_copter + 0.001f * pitch_acc;
//
//	uint8_t size = sprintf((char*) buffer, "\r\n%f,%f,%f,%f",roll_acc, pitch_acc,roll_copter,
//			pitch_copter);
//	usb_print(buffer, size);
//
//	/* Calculate elapsed time from last ac  */




//  /* Run Sensor Fusion algorithm */
  MotionFX_propagate(mfxstate, &data_out, &data_in, &dT);
  MotionFX_update(mfxstate, &data_out, &data_in, &dT, NULL);
  sensor_data.roll = data_out.rotation[1];
  sensor_data.pitch = data_out.rotation[2];
  sensor_data.yaw = data_out.rotation[0];

	if (ENABLE_6X == 1) {
		uint8_t size = sprintf((char*)buffer, "\r\n%f,%f,%f",sensor_data.roll,sensor_data.pitch,sensor_data.yaw );
		usb_print(buffer,size);
	}

}
