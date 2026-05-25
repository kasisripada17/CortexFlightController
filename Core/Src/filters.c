/*
 * filters.c
 *
 *  Created on: 10-May-2026
 *      Author: kasiviswanadhsripada
 */

#include"filters.h"
#include "math.h"

#define IMU_SAMPLE_RATE_HZ  1666.0f  // Your standard LSM6DS3 ODR
#define NOTCH_INIT_FREQ_HZ   200.0f  // Safe baseline starting frequency
#define NOTCH_DEFAULT_Q        1.5f  // Balanced width vs attenuation factorvolatile float current_fft_peak_hz = 200.0f; // Tracked by background task
BiquadNotch gyro_notch_x;
BiquadNotch gyro_notch_y;
BiquadNotch gyro_notch_z;


BiquadNotch acc_notch_x;
BiquadNotch acc_notch_y;
BiquadNotch acc_notch_z;

void LPF_Init(LPF_Filter *filter, float cutoff, float sampleRate) {
	float dt = 1.0f / sampleRate;
	float tau = 1.0f / (2.0f * 3.14159f * cutoff);
	filter->alpha = dt / (tau + dt);
	filter->outPrev = 0.0f;
}

float LPF_Update(LPF_Filter *filter, float input) {
	float output = filter->outPrev + filter->alpha * (input - filter->outPrev);
	filter->outPrev = output;
	return output;
}
// --- GLOBAL INSTANTIATIONS ---
volatile float current_fft_peak_hz = NOTCH_INIT_FREQ_HZ; // Set baseline frequency
volatile bool gyro_calib_done = false;                   // Tracks calibration status
float gyro_bias[3] = {0.0f, 0.0f, 0.0f};                 // Latched X, Y, Z offsets

// Dynamic Notch instances for each independent operational axis
BiquadNotch gyro_notch_x;
BiquadNotch gyro_notch_y;
BiquadNotch gyro_notch_z;

BiquadNotch acc_notch_x;
BiquadNotch acc_notch_y;
BiquadNotch acc_notch_z;
/**
 * @brief Zeroes delay registers and configures baseline starting coefficients.
 */
void Gyro_Notch_Filter_Init(void) {
    // Axis X (Roll) Reset
    gyro_notch_x.x1 = 0.0f; gyro_notch_x.x2 = 0.0f;
    gyro_notch_x.y1 = 0.0f; gyro_notch_x.y2 = 0.0f;
    Update_Notch_Coefficients(&gyro_notch_x, NOTCH_INIT_FREQ_HZ, IMU_SAMPLE_RATE_HZ, NOTCH_DEFAULT_Q);

    // Axis Y (Pitch) Reset
    gyro_notch_y.x1 = 0.0f; gyro_notch_y.x2 = 0.0f;
    gyro_notch_y.y1 = 0.0f; gyro_notch_y.y2 = 0.0f;
    Update_Notch_Coefficients(&gyro_notch_y, NOTCH_INIT_FREQ_HZ, IMU_SAMPLE_RATE_HZ, NOTCH_DEFAULT_Q);

    // Axis Z (Yaw) Reset
    gyro_notch_z.x1 = 0.0f; gyro_notch_z.x2 = 0.0f;
    gyro_notch_z.y1 = 0.0f; gyro_notch_z.y2 = 0.0f;
    Update_Notch_Coefficients(&gyro_notch_z, NOTCH_INIT_FREQ_HZ, IMU_SAMPLE_RATE_HZ, NOTCH_DEFAULT_Q);

    current_fft_peak_hz = NOTCH_INIT_FREQ_HZ;
    gyro_calib_done = false;





    // Axis X (Roll) Reset
    acc_notch_x.x1 = 0.0f; acc_notch_x.x2 = 0.0f;
    acc_notch_x.y1 = 0.0f; acc_notch_x.y2 = 0.0f;
    Update_Notch_Coefficients(&acc_notch_x, NOTCH_INIT_FREQ_HZ, IMU_SAMPLE_RATE_HZ, NOTCH_DEFAULT_Q);

    // Axis Y (Pitch) Reset
    acc_notch_y.x1 = 0.0f; acc_notch_y.x2 = 0.0f;
    acc_notch_y.y1 = 0.0f; acc_notch_y.y2 = 0.0f;
    Update_Notch_Coefficients(&acc_notch_y, NOTCH_INIT_FREQ_HZ, IMU_SAMPLE_RATE_HZ, NOTCH_DEFAULT_Q);

    // Axis Z (Yaw) Reset
    acc_notch_z.x1 = 0.0f; acc_notch_z.x2 = 0.0f;
    acc_notch_z.y1 = 0.0f; acc_notch_z.y2 = 0.0f;
    Update_Notch_Coefficients(&acc_notch_z, NOTCH_INIT_FREQ_HZ, IMU_SAMPLE_RATE_HZ, NOTCH_DEFAULT_Q);

    current_fft_peak_hz = NOTCH_INIT_FREQ_HZ;
    gyro_calib_done = false;
}

/**
 * @brief Computes continuous-to-discrete filter parameters pre-divided by a0.
 */
void Update_Notch_Coefficients(BiquadNotch *filter, float center_freq_hz, float sample_rate_hz, float Q) {
    // Boundary protection against illegal frequencies (Nyquist limit guard)
    if (center_freq_hz <= 10.0f || center_freq_hz >= (sample_rate_hz / 2.0f)) {
        return;
    }

    // Intermediary angular steps
    float omega = (2.0f * M_PI * center_freq_hz) / sample_rate_hz;
    float alpha = sinf(omega) / (2.0f * Q);
    float cos_w = cosf(omega);

    // Standard raw Biquad parameters
    float b0 = 1.0f;
    float b1 = -2.0f * cos_w;
    float b2 = 1.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cos_w;
    float a2 = 1.0f - alpha;

    // Normalize everything by a0 to avoid real-time division step inside ISR
    filter->b0 = b0 / a0;
    filter->b1 = b1 / a0;
    filter->b2 = b2 / a0;
    filter->a1 = a1 / a0;
    filter->a2 = a2 / a0;
}

/**
 * @brief High-speed Direct Form I difference execution line.
 */
inline float Run_Notch_Filter(BiquadNotch *f, float input) {
    // Execute difference equation
    float output = (f->b0 * input) + (f->b1 * f->x1) + (f->b2 * f->x2)
                   - (f->a1 * f->y1) - (f->a2 * f->y2);

    // Shift memory registers
    f->x2 = f->x1;
    f->x1 = input;
    f->y2 = f->y1;
    f->y1 = output;

    return output;
}
