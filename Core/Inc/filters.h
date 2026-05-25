/*
 * filters.h
 *
 *  Created on: 10-May-2026
 *      Author: kasiviswanadhsripada
 */

#ifndef INC_FILTERS_H_
#define INC_FILTERS_H_


typedef struct {
	float alpha;
	float outPrev;
} LPF_Filter;


// Function prototype
#define GYRO_SW_LPF
#define ACC_SW_LPF



void LPF_Init(LPF_Filter *filter, float cutoff, float sampleRate);
float LPF_Update(LPF_Filter *filter, float input);

#ifndef GYRO_FILTER_H
#define GYRO_FILTER_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// --- MATH CONSTANTS ---
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// --- IMU & CALIBRATION HARDWARE MACROS ---
#define IMU_SAMPLE_RATE_HZ       1666.0f   // LSM6DS3 standard ODR
#define GYRO_SCALE_DPS_PER_LSB   0.0175f   // 17.50 mdps/LSB for +/-500dps FSR
#define THERMAL_WARMUP_SAMPLES   3200      // ~1.92 seconds to skip thermal transient
#define CALIB_SNAPSHOT_SAMPLES   1666      // 1.00 second of flat, stable data to average

// --- DYNAMIC NOTCH CONFIGURATION ---
#define NOTCH_INIT_FREQ_HZ       200.0f    // Safe boot-up baseline target notch frequency
#define NOTCH_DEFAULT_Q          1.5f      // Quality factor (width of the attenuation notch)

// --- BIQUAD FILTER STRUCTURE ---
typedef struct {
    // Coefficients normalized by a0 (saves division cycles in high-speed loops)
    float b0, b1, b2;
    float a1, a2;

    // State Delay History Registers
    float x1, x2; // Input history
    float y1, y2; // Output history
} BiquadNotch;

// --- EXTERNAL GLOBAL VARIABLES ---
extern volatile bool gyro_calib_done;
extern float gyro_bias[3];
extern BiquadNotch gyro_notch_x;
extern BiquadNotch gyro_notch_y;
extern BiquadNotch gyro_notch_z;

// --- FUNCTION PROTOTYPES ---
void Gyro_Notch_Filter_Init(void);
void Update_Notch_Coefficients(BiquadNotch *filter, float center_freq_hz, float sample_rate_hz, float Q);
float Run_Notch_Filter(BiquadNotch *f, float input);

#endif // GYRO_FILTER_H


#endif /* INC_FILTERS_H_ */
