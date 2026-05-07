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
/* --- DJI-Style Velocity Altitude Control --- */
float target_climb_rate = 0.0f;
float alt_i_term = 0.0f;

// Gains for H7 at 416Hz
float Kp_vel_z = 12.0f; // High P-gain for snappy "locked-in" feel
float Ki_vel_z = 0.8f;  // Integral to maintain hover against weight

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

	integrate_global_position(dT) ;

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

/**
 * @brief Initialize the Notch Filter
 * @param centerFreq: The frequency to remove (the peak in your FFT)
 * @param bandwidth: How wide the "cut" is (start with 30.0f)
 * @param sampleFreq: Your STM32H7 loop frequency (e.g., 1000.0f or 8000.0f)
 */
void Notch_Init(BiquadNotch *filter, float centerFreq, float bandwidth,
		float sampleFreq) {
	float omega = 2.0f * M_PI * centerFreq / sampleFreq;
	float alpha = sinf(omega)
			* sinhf(logf(2.0f) / 2.0f * bandwidth * omega / sinf(omega));

	float a0 = 1.0f + alpha;
	filter->b0 = 1.0f / a0;
	filter->b1 = -2.0f * cosf(omega) / a0;
	filter->b2 = 1.0f / a0;
	filter->a1 = filter->b1;
	filter->a2 = (1.0f - alpha) / a0;

	// Reset history
	filter->x1 = filter->x2 = filter->y1 = filter->y2 = 0.0f;
}

/**
 * @brief Process new IMU data through the filter
 */
float Notch_Update(BiquadNotch *filter, float input) {
	// Standard Direct Form I Biquad Equation
	float output = filter->b0 * input + filter->b1 * filter->x1
			+ filter->b2 * filter->x2 - filter->a1 * filter->y1
			- filter->a2 * filter->y2;

	// Update history for next sample
	filter->x2 = filter->x1;
	filter->x1 = input;
	filter->y2 = filter->y1;
	filter->y1 = output;

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
    float a_global[3] = {0.0f,0.0f,0.0f};

    // Intermediate cross product terms
    float tx = 2.0f * (qy * az - qz * ay);
    float ty = 2.0f * (qz * ax - qx * az);
    float tz = 2.0f * (qx * ay - qy * ax);

    a_global[0] = ax + qw * tx + (qy * tz - qz * ty);
    a_global[1] = ay + qw * ty + (qz * tx - qx * tz);
    a_global[2] = az + qw * tz + (qx * ty - qy * tx);

    // 4. Integrate to Velocity
    for (int i = 0; i < 3; i++) {
        // Convert Gs to m/s^2 (Standard Gravity)
        float acc_ms2 = a_global[i] * 9.80665f;
        if(isnan(acc_ms2))
        {
        	acc_ms2=0.0f;
        }

        // Deadzone: Filter out remaining 250Hz motor vibrations
        // that escaped the hardware and software notch filters
        if (fabsf(acc_ms2) < 0.001f) {
            acc_ms2 = 0.0f;
        }

        // Integrate Acceleration -> Velocity
        velocity_earth[i] += acc_ms2 * dt;

        // "Leaky" Integrator: Essential for Inertial-only flight
        // without a Barometer or GPS to prevent velocity runaway.
        velocity_earth[i] *= 0.985f;

    }
}
// Global position in Earth Frame (Meters)
// [0]=North, [1]=East, [2]=Down

/**
 * @brief Integrates velocity to determine global position
 * @param dt: Sampling period (0.002403846f)
 */
void integrate_global_position(float dt) {
    for (int i = 0; i < 3; i++) {
        // Simple Euler Integration
        position_earth[i] += velocity_earth[i] * dt;

        /*
         * WARNING: We do NOT use a "Leak" on position.
         * If we leak position, the drone will always think its
         * starting point is moving toward it.
         * Position drift must be managed by the VELOCITY leak.
         */
    }
}



