/*
 * motion_FX.c
 *
 *  Created on: 19-Apr-2026
 *      Author: kasiviswanadhsripada
 */
#include "stm32h7xx_hal.h"
#include "lsm6ds3.h"
#include "sensor_fusion.h"
#include "print.h"
#include "stdbool.h"
#include "filters.h"
/*********************************/
/*ST's Motion FX LIB related data*/

float accx = 0.0f, accy = 0.0f, accz = 0.0f;
float angx = 0.0f, angy = 0.0f, angz = 0.0f;

extern float gyro_filteredx, gyro_filteredy, gyro_filteredz;

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
void Mahony_GetEulerAngles(MahonyFilter_t *filter, float *roll, float *pitch,
		float *yaw);
void Mahony_GetQuaternion(MahonyFilter_t *filter, float *q0, float *q1,
		float *q2, float *q3);
void calculate_linear_acceleration(void) ;

MahonyFilter_t imu = { 0 };


extern BiquadNotch gyro_notch_x;
extern BiquadNotch gyro_notch_y;
extern BiquadNotch gyro_notch_z;

extern BiquadNotch acc_notch_x;
extern BiquadNotch acc_notch_y;
extern BiquadNotch acc_notch_z;

LPF_Filter gyroFilterX, gyroFilterY, gyroFilterZ;
LPF_Filter accFilterX, accFilterY, accFilterZ;

void convert_local_to_globalframe(MahonyFilter_t *filter,float dt);
float position_earth[3] = { 0.0f, 0.0f, 0.0f };
float velocity_earth[3] = { 0.0f, 0.0f, 0.0f };
void integrate_global_position(float dt);
/* --- Velocity Control Variables --- */
float target_position[3] = { 0.0f, 0.0f, 0.0f };
float target_velocity[3] = { 0.0f, 0.0f, 0.0f };


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
void sensor_fusion_update(void) {

	static bool init = 1;
	if (init) {
		init = 0;
#ifdef GYRO_SW_LPF

		LPF_Init(&gyroFilterX, 80.0f, LOOP_FREQ);
		LPF_Init(&gyroFilterY, 80.0f, LOOP_FREQ);
		LPF_Init(&gyroFilterZ, 80.0f, LOOP_FREQ);
#endif

#ifdef ACC_SW_LPF
		LPF_Init(&accFilterX, 5.0f, LOOP_FREQ);
		LPF_Init(&accFilterY, 5.0f, LOOP_FREQ);
		LPF_Init(&accFilterZ, 5.0f, LOOP_FREQ);
#endif

		Gyro_Notch_Filter_Init();

		Mahony_Init(&imu, LOOP_FREQ, 0.15f, 0.001f);
	}


	float roll = 0.0f, pitch = 0.0f,yaw = 0.0f;

	// Run through the dynamic notch processors
	gyro_filteredx = Run_Notch_Filter(&gyro_notch_x, sensor_data.gyro_cal_x);
	gyro_filteredy = Run_Notch_Filter(&gyro_notch_y, sensor_data.gyro_cal_y);
	gyro_filteredz = Run_Notch_Filter(&gyro_notch_z, sensor_data.gyro_cal_z);

	// Run through the dynamic notch processors
	float ax_filtered = Run_Notch_Filter(&acc_notch_x, sensor_data.acc_x);
	float ay_filtered = Run_Notch_Filter(&acc_notch_y, sensor_data.acc_y);
	float az_filtered = Run_Notch_Filter(&acc_notch_z, sensor_data.acc_z);

#ifdef GYRO_SW_LPF

	angx = LPF_Update(&gyroFilterX, gyro_filteredx);
	angy = LPF_Update(&gyroFilterY, gyro_filteredy);
	angz = LPF_Update(&gyroFilterZ, gyro_filteredz);

#else
	angx = sensor_data.gyro_cal_x; // Pass raw data if LPF is disabled
	angy = sensor_data.gyro_cal_y; // Pass raw data if LPF is disabled
	angz = sensor_data.gyro_cal_z; // Pass raw data if LPF is disabled

#endif

#ifdef ACC_SW_LPF
	accx = LPF_Update(&accFilterX, ax_filtered);
	accy = LPF_Update(&accFilterY, ay_filtered);
	accz = LPF_Update(&accFilterZ, az_filtered);

#else
	accx = sensor_data.acc_x;
	accy = sensor_data.acc_y;
	accz = sensor_data.acc_z;
#endif

#ifdef MAHONY_AHRS
	// Update filter
	Mahony_UpdateIMU(&imu, angx, angy, angz, accx, accy, accz);

	Mahony_GetEulerAngles(&imu, &roll, &pitch, &yaw);
#endif

	sensor_data.roll = roll;
	sensor_data.pitch = pitch;



// calculate_linear_acceleration() ;

}




// Fast inverse square root (optional, but Betaflight uses it)
static float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*) &y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*) &i;
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
    float vx, vy, vz;
    float ex, ey, ez;
    float qa, qb, qc, qd;

    // 1. Convert gyro degrees/sec to radians/sec
    gx *= 0.0174532925f;
    gy *= 0.0174532925f;
    gz *= 0.0174532925f;

    // 2. Only calculate accelerometer feedback if vector is non-zero
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalize accelerometer measurement
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 3. Corrected Estimated direction of gravity (v) from current quaternion orientation
        vx = 2.0f * (filter->q1 * filter->q3 - filter->q0 * filter->q2);
        vy = 2.0f * (filter->q0 * filter->q1 + filter->q2 * filter->q3);
        vz = filter->q0 * filter->q0 - filter->q1 * filter->q1 - filter->q2 * filter->q2 + filter->q3 * filter->q3;

        // 4. Error is the cross product between estimated and measured gravity vectors
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);

        // 5. Accumulate Integral feedback cleanly if Ki > 0
        if (filter->Ki > 0.0f) {
            filter->integralFBx += filter->Ki * ex * filter->invSampleFreq;
            filter->integralFBy += filter->Ki * ey * filter->invSampleFreq;
            filter->integralFBz += filter->Ki * ez * filter->invSampleFreq;

            // Apply integral term to a local tracking parameter, NOT mutating gx directly yet
            gx += filter->integralFBx;
            gy += filter->integralFBy;
            gz += filter->integralFBz;
        } else {
            filter->integralFBx = 0.0f;
            filter->integralFBy = 0.0f;
            filter->integralFBz = 0.0f;
        }

        // 6. Apply Proportional Feedback (Kp) directly to tracking variables
        gx += filter->Kp * ex;
        gy += filter->Kp * ey;
        gz += filter->Kp * ez;
    }

    // 7. Integrate Rate of Change of Quaternion (True Simultaneous Derivation)
    float delta_t = 0.5f * filter->invSampleFreq;
    qa = filter->q0;
    qb = filter->q1;
    qc = filter->q2;
    qd = filter->q3;

    // Compute explicit step updates locally to avoid state cross-contamination
    float dq0 = (-qb * gx - qc * gy - qd * gz) * delta_t;
    float dq1 = ( qa * gx + qc * gz - qd * gy) * delta_t;
    float dq2 = ( qa * gy - qb * gz + qd * gx) * delta_t;
    float dq3 = ( qa * gz + qb * gy - qc * gx) * delta_t;

    filter->q0 += dq0;
    filter->q1 += dq1;
    filter->q2 += dq2;
    filter->q3 += dq3;

    // 8. Normalize Quaternion to prevent numeric explosion
    recipNorm = 1.0f / sqrtf(filter->q0 * filter->q0 + filter->q1 * filter->q1 +
                             filter->q2 * filter->q2 + filter->q3 * filter->q3);
    filter->q0 *= recipNorm;
    filter->q1 *= recipNorm;
    filter->q2 *= recipNorm;
    filter->q3 *= recipNorm;
}

void Mahony_GetEulerAngles(MahonyFilter_t *filter, float *roll, float *pitch,
		float *yaw) {
	float q0 = filter->q0;
	float q1 = filter->q1;
	float q2 = filter->q2;
	float q3 = filter->q3;

	// Roll (rotation around X-axis)
	float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
	float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
	*roll = atan2f(sinr_cosp, cosr_cosp) * 57.2957795f;

	// Pitch (rotation around Y-axis) - Standard, safe arcsine method
	float sinp = 2.0f * (q0 * q2 - q3 * q1);
	if (sinp > 1.0f)
		sinp = 1.0f;
	if (sinp < -1.0f)
		sinp = -1.0f;
	*pitch = asinf(sinp) * 57.2957795f;

	// Yaw (rotation around Z-axis)
	float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
	float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
	*yaw = atan2f(siny_cosp, cosy_cosp) * 57.2957795f;
}
void Mahony_GetQuaternion(MahonyFilter_t *filter, float *q0, float *q1,
		float *q2, float *q3) {
	*q0 = filter->q0;
	*q1 = filter->q1;
	*q2 = filter->q2;
	*q3 = filter->q3;
}

void calculate_linear_acceleration(void) {
    // 1. Define gravity constant (m/s^2)
    const float GRAVITY = 9.80665f;
    // Degree to Radian conversion factor (pi / 180)
    const float DEG_TO_RAD = 0.0174532925f;

    // 2. Convert raw accelerometer readings from Gs to m/s^2
    Vector3f_t a_body;
    a_body.x = sensor_data.acc_x * GRAVITY;
    a_body.y = sensor_data.acc_y * GRAVITY;
    a_body.z = sensor_data.acc_z * GRAVITY;

    // 3. Convert attitude from Degrees to RADIANS (FIXED)
    float roll_rad  = sensor_data.roll * DEG_TO_RAD;
    float pitch_rad = sensor_data.pitch * DEG_TO_RAD;
    float yaw_rad   = sensor_data.yaw * DEG_TO_RAD;

    float cos_r = cosf(roll_rad);
    float sin_r = sinf(roll_rad);

    // FIXED: cos_p was accidentally using sinf() in your original code
    float cos_p = cosf(pitch_rad);
    float sin_p = sinf(pitch_rad);

    float cos_y = cosf(yaw_rad);
    float sin_y = sinf(yaw_rad);

    // 4. Rotate Body-Frame Accelerations into Earth-Frame (NED / Navigation Frame)
    Vector3f_t a_earth;

    a_earth.x = a_body.x * (cos_p * cos_y) +
                a_body.y * (sin_r * sin_p * cos_y - cos_r * sin_y) +
                a_body.z * (cos_r * sin_p * cos_y + sin_r * sin_y);

    a_earth.y = a_body.x * (cos_p * sin_y) +
                a_body.y * (sin_r * sin_p * sin_y + cos_r * cos_y) +
                a_body.z * (cos_r * sin_p * sin_y - sin_r * cos_y);

    a_earth.z = a_body.x * (-sin_p) +
                a_body.y * (sin_r * cos_p) +
                a_body.z * (cos_r * cos_p);

    // 5. REMOVE GRAVITY FROM THE EARTH Z-AXIS
    // If your IMU reads +1G sitting flat on a table pointing UP, a_earth.z is +9.81.
    // Subtracting GRAVITY yields 0.0 m/s^2 at perfect rest.
    float linear_accel_earth_x = a_earth.x;
    float linear_accel_earth_y = a_earth.y;
    float linear_accel_earth_z = a_earth.z - GRAVITY;

    // 6. Pass clean values to state variables
    sensor_data.acc_earth_x = linear_accel_earth_x;
    sensor_data.acc_earth_y = linear_accel_earth_y;
    sensor_data.acc_earth_z = linear_accel_earth_z;
}




