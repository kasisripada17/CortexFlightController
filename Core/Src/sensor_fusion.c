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
#define GYRO_SW_LPF
#define ACC_SW_LPF
//#define ACC_NOTCH_FILTER
char lib_version_mfx[MFX_STR_LENG];
static uint8_t mfxstate[STATE_SIZE ];
void convert_local_to_globalframe(MFX_output_t *data_out, float dt) ;
float position_earth[3] = {0.0f, 0.0f, 0.0f};
float velocity_earth[3] = {0.0f, 0.0f, 0.0f};
void integrate_global_position(float dt) ;
/* --- Velocity Control Variables --- */
float target_position[3] = {0.0f, 0.0f, 0.0f};
float target_velocity[3] = {0.0f, 0.0f, 0.0f};

static float fused_alt = 0.0f;
static float fused_vel = 0.0f;
float a_global[3] = {0.0f,0.0f,0.0f};

#define HOVER_THROTTLE 1500
#define STICK_DEADZONE 0.1f

MFX_knobs_t iKnobs;
float LastTime;
extern volatile IMU_Data_t sensor_data;
typedef struct {
	float x1, x2;      // state
	float a1, a2, b0;  // coefficients
} PT2Filter;

PT2Filter gyro_filter[3];
PT2Filter acc_filter[3];

typedef struct {
	float alpha;
	float outPrev;
} LPF_Filter;
static LPF_Filter gyroFilterX, gyroFilterY, gyroFilterZ;
static LPF_Filter accFilterX, accFilterY, accFilterZ;

typedef struct {
	float b0, b1, b2; // Numerator coefficients
	float a1, a2;     // Denominator coefficients
	float x1, x2;     // Input history
	float y1, y2;     // Output history
} BiquadNotch;

float Notch_Update(BiquadNotch *filter, float input);
void Notch_Init(BiquadNotch *filter, float centerFreq, float bandwidth,
		float sampleFreq);
BiquadNotch notchX, notchY,notchZ;


// Initialize filter (Targeting 80Hz cutoff at 416Hz ODR)
void LPF_Init(LPF_Filter *filter, float cutoff, float sampleRate);

// Apply filter
float LPF_Update(LPF_Filter *filter, float input);

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
//	iKnobs.ATime = 5.0f;
//	iKnobs.FrTime =20.0f;
	iKnobs.LMode = 0;
	iKnobs.modx = 1;
	iKnobs.acc_orientation[0] = 'w';
	iKnobs.acc_orientation[1] = 's';
	iKnobs.acc_orientation[2] = 'u';
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
#ifdef GYRO_SW_LPF

	LPF_Init(&gyroFilterX, 80.0f, 416.0f);
	LPF_Init(&gyroFilterY, 80.0f, 416.0f);
	LPF_Init(&gyroFilterZ, 80.0f, 416.0f);
#endif

#ifdef ACC_SW_LPF
	LPF_Init(&accFilterX, 30.0f, 416.0f);
	LPF_Init(&accFilterY, 30.0f, 416.0f);
	LPF_Init(&accFilterZ, 30.0f, 416.0f);
#endif


}

/* Using Sensor Fusion algorithm */
void motion_fx_update(void) {
	MFX_input_t data_in;
	MFX_output_t data_out;
	float dT = 0.002403846f;

float accx = 0.0f,accy = 0.0f,accz = 0.0f;
float angx = 0.0f,angy = 0.0f,angz = 0.0f;


#ifdef GYRO_SW_LPF


	angx = LPF_Update(&gyroFilterX, sensor_data.gyro_cal_x);
	angy = LPF_Update(&gyroFilterY, sensor_data.gyro_cal_y);
	angz = LPF_Update(&gyroFilterZ, sensor_data.gyro_cal_z);
#else
	angx = sensor_data.gyro_cal_x; // Pass raw data if LPF is disabled
	angy = sensor_data.gyro_cal_y; // Pass raw data if LPF is disabled
	angz = sensor_data.gyro_cal_z; // Pass raw data if LPF is disabled
#endif





#ifdef ACC_SW_LPF
	accx = LPF_Update(&accFilterX, sensor_data.acc_x);
	accy = LPF_Update(&accFilterY, sensor_data.acc_y);
	accz = LPF_Update(&accFilterZ, sensor_data.acc_z);

#else
	accx = sensor_data.acc_x;
	accy = sensor_data.acc_y;
	accz = sensor_data.acc_z;
#endif


	data_in.gyro[0] = angx;
	data_in.gyro[1] = angy;
	data_in.gyro[2] = angz;


	data_in.acc[0] = accx;
	data_in.acc[1] = accy;
	data_in.acc[2] = accz;



//	uint8_t size = sprintf((char*) buffer, "\r\n%f,%f,%f,%f,%f,%f",
//			data_in.gyro[0],data_in.gyro[1],
//			data_in.gyro[2],data_in.acc[0],data_in.acc[1],data_in.acc[2]);
//	usb_print(buffer, size);



  /* Run Sensor Fusion algorithm */
	MotionFX_propagate(mfxstate, &data_out, &data_in, &dT);
	MotionFX_update(mfxstate, &data_out, &data_in, &dT, NULL);
	sensor_data.roll = data_out.rotation[1];
	sensor_data.pitch = data_out.rotation[2];
	sensor_data.yaw = data_out.rotation[0];
//
//	uint8_t size = sprintf((char*) buffer, "\r\n%f,%f,%f",
//			sensor_data.roll,sensor_data.pitch,sensor_data.yaw);
//	usb_print(buffer, size);


	convert_local_to_globalframe(&data_out,  dT) ;


}

// Initialize filter (Targeting 80Hz cutoff at 416Hz ODR)
void LPF_Init(LPF_Filter *filter, float cutoff, float sampleRate) {
	float dt = 1.0f / sampleRate;
	float tau = 1.0f / (2.0f * 3.14159f * cutoff);
	filter->alpha = dt / (tau + dt);
	filter->outPrev = 0.0f;
}

// Apply filter
float LPF_Update(LPF_Filter *filter, float input) {
	float output = filter->outPrev + filter->alpha * (input - filter->outPrev);
	filter->outPrev = output;
	return output;
}



void convert_local_to_globalframe(MFX_output_t *data_out, float dt) {
    // 1. Extract Quaternion (q0=w, q1=x, q2=y, q3=z)
    float qw = data_out->quaternion[0];
    float qx = data_out->quaternion[1];
    float qy = data_out->quaternion[2];
    float qz = data_out->quaternion[3];

    // 2. Extract Body Linear Acceleration (Gs)
    float ax = data_out->linear_acceleration[0];
    float ay = data_out->linear_acceleration[1];
    float az = data_out->linear_acceleration[2];




    // 3. Apply Quaternion Rotation: v_global = q * v_local * q_conj
    // Using the optimized Hamilton product expansion

    // Intermediate cross product terms
    float tx = 2.0f * (qy * az - qz * ay);
    float ty = 2.0f * (qz * ax - qx * az);
    float tz = 2.0f * (qx * ay - qy * ax);

    a_global[0] = ax + qw * tx + (qy * tz - qz * ty);
    a_global[1] = ay + qw * ty + (qz * tx - qx * tz);
    a_global[2] = az + qw * tz + (qx * ty - qy * tx);

}


float  update_altitude_fusion(float baro_alt, float acc_z_earth, float dt) {
	const float M_TO_FT = 3.28084f;
	float acc_z_ft = a_global[2] * M_TO_FT;
    // 1. Predict state using Accelerometer
    fused_vel += acc_z_earth * dt;
    fused_alt += fused_vel * dt + 0.5f * acc_z_earth * dt * dt;

    // 2. Calculate Error (Baro vs. Prediction)
    float error = baro_alt - fused_alt;

    // 3. Correct the state (Tuning constants: 0.1 and 0.01 are good starts)
    fused_alt += error * 0.1f;    // Pulls altitude toward baro
    fused_vel += error * 0.01f;   // Pulls velocity toward baro
    return fused_alt;
}

