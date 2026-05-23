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
#include "stdbool.h"
#include "filters.h"
/*********************************/
/*ST's Motion FX LIB related data*/

//#define MOTION_FX_ST
#ifdef MOTION_FX_ST
#define MFX_STR_LENG 35
#define STATE_SIZE (uint32_t)(2450)
#define ENABLE_6X 1
char lib_version_mfx[MFX_STR_LENG];
static uint8_t mfxstate[STATE_SIZE ];
MFX_knobs_t iKnobs;
#endif










#define MAHONY_AHRS

// Mahony filter structure
typedef struct {
    float q0, q1, q2, q3;  // Quaternion of sensor frame relative to earth frame
    float integralFBx, integralFBy, integralFBz;  // Integral error terms
    float Kp;  // Proportional gain
    float Ki;  // Integral gain
    float invSampleFreq;  // 1 / sample frequency
} MahonyFilter_t;

// Function prototypes
void Mahony_Init(MahonyFilter_t *filter, float sampleFreq, float Kp, float Ki);
void Mahony_Update(MahonyFilter_t *filter, float gx, float gy, float gz,
                   float ax, float ay, float az);
void Mahony_UpdateIMU(MahonyFilter_t *filter, float gx, float gy, float gz,
                      float ax, float ay, float az);
void Mahony_GetEulerAngles(MahonyFilter_t *filter, float *roll, float *pitch, float *yaw);
void Mahony_GetQuaternion(MahonyFilter_t *filter, float *q0, float *q1, float *q2, float *q3);
MahonyFilter_t imu = {0};

float roll, pitch, yaw;






LPF_Filter gyroFilterX, gyroFilterY, gyroFilterZ;
LPF_Filter accFilterX, accFilterY, accFilterZ;


void convert_local_to_globalframe(MFX_output_t *data_out, float dt);
float position_earth[3] = { 0.0f, 0.0f, 0.0f };
float velocity_earth[3] = { 0.0f, 0.0f, 0.0f };
void integrate_global_position(float dt);
/* --- Velocity Control Variables --- */
float target_position[3] = { 0.0f, 0.0f, 0.0f };
float target_velocity[3] = { 0.0f, 0.0f, 0.0f };

static float fused_alt = 0.0f;
static float fused_vel = 0.0f;
float a_global[3] = { 0.0f, 0.0f, 0.0f };

#define HOVER_THROTTLE 1500
#define STICK_DEADZONE 0.1f
float LastTime;
extern volatile IMU_Data_t sensor_data;

typedef struct {
	float w, x, y, z;
} quaternion_t;

// Global IMU state
quaternion_t q = { 1.0f, 0.0f, 0.0f, 0.0f };
float rMat[3][3]; // Rotation matrix derived from q

void imuMahonyUpdate(float dt, float gx, float gy, float gz, float ax, float ay,
		float az);
void imuComputeRotationMatrix(void);
void getEulerAngles(float *roll, float *pitch);

extern uint8_t buffer[256];
//	/* USER CODE END 2 */
float roll_copter = 0.0f;
float pitch_copter = 0.0f;

#ifdef MOTION_FX_ST
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
//	iKnobs.ATime = 0.8f;
//	iKnobs.FrTime =0.8f;
	//iKnobs.MTime=1.0f;
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



}
#endif
/* Using Sensor Fusion algorithm */
void motion_fx_update(void) {

	static bool init = 1;
	if (init) {
		init = 0;
#ifdef GYRO_SW_LPF

		LPF_Init(&gyroFilterX, 100.0f, LOOP_FREQ);
		LPF_Init(&gyroFilterY, 100.0f, LOOP_FREQ);
		LPF_Init(&gyroFilterZ, 100.0f,LOOP_FREQ);
#endif

#ifdef ACC_SW_LPF
		LPF_Init(&accFilterX, 50.0f, LOOP_FREQ);
		LPF_Init(&accFilterY, 50.0f, LOOP_FREQ);
		LPF_Init(&accFilterZ, 50.0f, LOOP_FREQ);
#endif
		Mahony_Init(&imu, LOOP_FREQ, 2.5f, 0.0f);
	}

	float dt = DT;

	static float accx = 0.0f, accy = 0.0f, accz = 0.0f;
	static float angx = 0.0f, angy = 0.0f, angz = 0.0f;
	float roll = 0.0f,pitch=0.0f;

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
#ifdef MAHONY_AHRS
   // // Update filter
   // Mahony_UpdateIMU(&imu,  angx, angy, angz, -accx,-accy, accz);
	Mahony_UpdateIMU(&imu,  angx, angy, angz, accx,accy, accz);

    Mahony_GetEulerAngles(&imu, &roll, &pitch, &yaw);

//
//	uint8_t size = sprintf((char*) buffer, "\r\n%f,%f,%f,%f,%f,%f",
//			 angx, angy, angz, accx, accy, accz);
//	usb_print(buffer, size);
#endif
#ifdef MOTION_FX_ST
	MFX_input_t data_in;
	MFX_output_t data_out;

	data_in.gyro[0] = angx;
	data_in.gyro[1] = angy;
	data_in.gyro[2] = angz;
	data_in.acc[0] = accx;
	data_in.acc[1] = accy;
	data_in.acc[2] = accz;

	/* Run Sensor Fusion algorithm */
	MotionFX_propagate(mfxstate, &data_out, &data_in, &dt);
	MotionFX_update(mfxstate, &data_out, &data_in, &dt, NULL);
	sensor_data.roll = data_out.rotation[1];
	sensor_data.pitch = data_out.rotation[2];
	sensor_data.yaw = data_out.rotation[0];
#endif

	sensor_data.roll = roll;
	sensor_data.pitch = pitch ;


//
//
//	uint8_t size = sprintf((char*) buffer, "\r\n%f,%f",
//			roll,pitch);
//	usb_print(buffer, size);
//


//convert_local_to_globalframe(&data_out,  dT) ;

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


float update_altitude_fusion(float baro_alt, float acc_z_earth, float dt) {
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











// Fast inverse square root (optional, but Betaflight uses it)
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void Mahony_Init(MahonyFilter_t *filter, float sampleFreq, float Kp, float Ki) {
    filter->q0 = 1.0f;
    filter->q1 = 0.0f;
    filter->q2 = 0.0f;
    filter->q3 = 0.0f;
    filter->integralFBx = 0.0f;
    filter->integralFBy = 0.0f;
    filter->integralFBz = 0.0f;

    filter->Kp = Kp;
    filter->Ki = Ki;
    filter->invSampleFreq = 1.0f / sampleFreq;
}
void Mahony_UpdateIMU(MahonyFilter_t *filter, float gx, float gy, float gz,
                      float ax, float ay, float az) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;

    // 1. Convert gyro degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // 2. Compute feedback only if accelerometer measurement is valid
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalize accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 3. Estimated direction of gravity (v) based on current quaternion
        halfvx = filter->q1 * filter->q3 - filter->q0 * filter->q2;
        halfvy = filter->q0 * filter->q1 + filter->q2 * filter->q3;
        halfvz = filter->q0 * filter->q0 - 0.5f + filter->q3 * filter->q3;

        // 4. Error is cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 5. Apply Integral Feedback (Ki)
        if (filter->Ki > 0.0f) {
            filter->integralFBx += filter->Ki * halfex * filter->invSampleFreq;
            filter->integralFBy += filter->Ki * halfey * filter->invSampleFreq;
            filter->integralFBz += filter->Ki * halfez * filter->invSampleFreq;
            gx += filter->integralFBx;
            gy += filter->integralFBy;
            gz += filter->integralFBz;
        } else {
            filter->integralFBx = 0.0f;
            filter->integralFBy = 0.0f;
            filter->integralFBz = 0.0f;
        }

        // 6. Apply Proportional Feedback (Kp)
        gx += filter->Kp * halfex;
        gy += filter->Kp * halfey;
        gz += filter->Kp * halfez;
    }

    // 7. Integrate Rate of Change of Quaternion (Hamiltonian Derivation)
    float delta_t = 0.5f * filter->invSampleFreq;

    // Buffer current states to ensure simultaneous update
    float q0 = filter->q0;
    float q1 = filter->q1;
    float q2 = filter->q2;
    float q3 = filter->q3;

    filter->q0 += (-q1 * gx - q2 * gy - q3 * gz) * delta_t;
    filter->q1 += ( q0 * gx + q2 * gz - q3 * gy) * delta_t;
    filter->q2 += ( q0 * gy - q1 * gz + q3 * gx) * delta_t;
    filter->q3 += ( q0 * gz + q1 * gy - q2 * gx) * delta_t;

    // 8. CRITICAL: Normalize Quaternion to prevent drift/explosion
    recipNorm = invSqrt(filter->q0 * filter->q0 + filter->q1 * filter->q1 +
                        filter->q2 * filter->q2 + filter->q3 * filter->q3);
    filter->q0 *= recipNorm;
    filter->q1 *= recipNorm;
    filter->q2 *= recipNorm;
    filter->q3 *= recipNorm;
}
void Mahony_GetEulerAngles(MahonyFilter_t *filter, float *roll, float *pitch, float *yaw) {
    float q0 = filter->q0;
    float q1 = filter->q1;
    float q2 = filter->q2;
    float q3 = filter->q3;

    // Roll (rotation around X-axis) - using atan2
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    *roll = atan2f(sinr_cosp, cosr_cosp) * 57.2957795f;

    // Pitch (rotation around Y-axis) - using atan2 instead of asin
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    float cosp = 1.0f - 2.0f * (q2 * q2);
    *pitch = atan2f(sinp, cosp) * 57.2957795f;

    // Yaw (rotation around Z-axis) - using atan2
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    *yaw = atan2f(siny_cosp, cosy_cosp) * 57.2957795f;
}

void Mahony_GetQuaternion(MahonyFilter_t *filter, float *q0, float *q1, float *q2, float *q3) {
    *q0 = filter->q0;
    *q1 = filter->q1;
    *q2 = filter->q2;
    *q3 = filter->q3;
}
