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
static uint8_t mfxstate[STATE_SIZE];
MFX_knobs_t iKnobs;
float LastTime;
extern volatile IMU_Data_t sensor_data ;
extern uint8_t buffer[256];

void motionfx_init(void)
{
	/* Check if statically allocated memory size is sufficient
	 to store MotionFX algorithm state and resize if necessary */
	if (STATE_SIZE < MotionFX_GetStateSize())
	{
		  __disable_irq();
		  while (1)
		  {
		  }
	}

	/* Sensor Fusion API initialization function */
	MotionFX_initialize((MFXState_t *)mfxstate);
	/* Optional: Get version */
	MotionFX_GetLibVersion(lib_version_mfx);
	/* Modify knobs settings & set the knobs */
	MotionFX_getKnobs(mfxstate, &iKnobs);
	// 2. Adjust for a Drone (High vibration environment)
	iKnobs.ATime = 5.0f;        // Increase this if the horizon "shakes" during flight
	iKnobs.MTime = 2.0f;        // Snap to heading quickly
	iKnobs.LMode = 2;           // High-dynamic mode for aircraft
	iKnobs.modx = 1;            // Enable Gyro Bias estimation within the EKF

	// 3. Set the coordinate system
	// Most flight controllers use NED (North East Down)
	iKnobs.output_type = MFX_ENGINE_OUTPUT_NED;
	MotionFX_setKnobs(mfxstate, &iKnobs);
	MotionFX_enable_6X(mfxstate, MFX_ENGINE_DISABLE);
	MotionFX_enable_9X(mfxstate, MFX_ENGINE_DISABLE);
	/* Enable 9-axis sensor fusion */
	if (ENABLE_6X == 1)
	{
	 MotionFX_enable_6X(mfxstate, MFX_ENGINE_ENABLE);
	}
	else
	{
	 MotionFX_enable_9X(mfxstate, MFX_ENGINE_ENABLE);
	}
}


/* Using Sensor Fusion algorithm */
void motion_fx_update(void)
{
 MFX_input_t data_in;
 MFX_output_t data_out;
 float  dT = 0.002403846f;


 data_in.acc[0] = sensor_data.acc_x;
 data_in.acc[1] = sensor_data.acc_y;
 data_in.acc[2] = sensor_data.acc_z;

 data_in.gyro[0] = sensor_data.gyro_cal_x;
 data_in.gyro[1] = sensor_data.gyro_cal_y;
 data_in.gyro[2] = sensor_data.gyro_cal_z;


 /* Calculate elapsed time from last ac  */



  /* Run Sensor Fusion algorithm */
  MotionFX_propagate(mfxstate, &data_out, &data_in, &dT);
  MotionFX_update(mfxstate, &data_out, &data_in, &dT, NULL);
  sensor_data.roll = data_out.rotation[2];
  sensor_data.pitch = data_out.rotation[1];
  sensor_data.yaw = data_out.rotation[0];


  if (ENABLE_6X == 1)
  {
  /* Game rotation Vector */
//		uint8_t size = sprintf((char*)buffer, "\r\n%f,%f,%f",data_out.rotation[0],data_out.rotation[1],data_out.rotation[2]);
//		usb_print(buffer,size);
  }

 }
