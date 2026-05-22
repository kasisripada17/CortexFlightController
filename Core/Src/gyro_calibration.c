/*
 * sensor_fusion.c
 *
 *  Created on: 18-Apr-2026
 *      Author: kasiviswanadhsripada
 */
#include "gyro_calibration.h"
#include "lsm6ds3.h"
#include <stdio.h>
#include "print.h"
#include "flight_control.h"
#include <string.h>


#define VERSION_STR_LENG 35
#define SAMPLE_FREQUENCY LOOP_FREQ

/* Initialization */
char lib_version[VERSION_STR_LENG];

MGC_knobs_t knobs;
MGC_output_t start_gyro_bias;
float sample_freq;

extern arm_state_t armed_status;
extern volatile IMU_Data_t sensor_data;
extern uint8_t buffer[256];
extern uint16_t size;
void gyro_calibration_init(void) {
	sample_freq = LOOP_FREQ;
	MotionGC_Initialize(MGC_MCU_STM32, &sample_freq);

	/* Optional: Get version */
	MotionGC_GetLibVersion(lib_version);

	/* Gyroscope calibration API initialization function */

	/* Optional: Get knobs settings */
	MotionGC_GetKnobs(&knobs);
	/* Optional: Adjust knobs settings */
	// Increase Acc threshold to 20mg (Standard is often 0.01 or 0.02)
	knobs.AccThr = 0.2f;     // Increased from 0.05
	knobs.GyroThr = 10.0f;    // Increased from 2.0
	// Ensure FastStart is enabled (yours is 1, which is good)
	knobs.FastStart = 1;
//	load_gyro_seed_from_flash();
//	/* Optional: Set initial gyroscope offset */

	start_gyro_bias.GyroBiasX = 0.705f;
	start_gyro_bias.GyroBiasY = -7.0f;
	start_gyro_bias.GyroBiasZ = -1.2f;
    MotionGC_SetCalParams(&start_gyro_bias);

	MotionGC_SetKnobs(&knobs);

	/* Optional: Set sample frequency */
	MotionGC_SetFrequency(&sample_freq);
}

/* Using gyroscope calibration algorithm */
void gyro_calibration_routine() {
	MGC_input_t data_in = { 0 };
	static MGC_output_t data_out = { 0 };
	int bias_update = 0;
	data_in.Gyro[0] = sensor_data.gyro_x;
	data_in.Gyro[1] = sensor_data.gyro_y;
	data_in.Gyro[2] = sensor_data.gyro_z;
	data_in.Acc[0] = sensor_data.acc_x;
	data_in.Acc[1] = sensor_data.acc_y;
	data_in.Acc[2] = sensor_data.acc_z;


	if (armed_status != ARMED) {
		/* Gyroscope calibration algorithm update */
		MotionGC_Update(&data_in, &data_out, &bias_update);
	//	process_motion_gc_autosave(&data_out,bias_update);
	}
	/* Apply correction */
	sensor_data.gyro_cal_x = (data_in.Gyro[0] - data_out.GyroBiasX);
	sensor_data.gyro_cal_y = (data_in.Gyro[1] - data_out.GyroBiasY);
	sensor_data.gyro_cal_z = (data_in.Gyro[2] - data_out.GyroBiasZ);


}

void load_gyro_seed_from_flash(void) {
    calibration_registry_t flash_data;
    MGC_output_t initial_bias;

    // Direct memory-mapped read from STM32 Flash
    memcpy(&flash_data, (uint32_t*)CALIB_FLASH_ADDR, sizeof(calibration_registry_t));

    // Calculate a simple checksum verification
    uint32_t calculated_checksum = (uint32_t)(flash_data.gyro_bias_x * 100) +
                                   (uint32_t)(flash_data.gyro_bias_y * 100) +
                                   (uint32_t)(flash_data.gyro_bias_z * 100);

    // Validate if sector is programmed and intact
    if (flash_data.magic == FLASH_MAGIC_NUMBER && flash_data.checksum == calculated_checksum) {
        initial_bias.GyroBiasX = flash_data.gyro_bias_x;
        initial_bias.GyroBiasY = flash_data.gyro_bias_y;
        initial_bias.GyroBiasZ = flash_data.gyro_bias_z;
    } else {
        // No valid profile found: seed with clean slate (defaults to zero)
        initial_bias.GyroBiasX = 0.0f;
        initial_bias.GyroBiasY = 0.0f;
        initial_bias.GyroBiasZ = 0.0f;
    }

    // Pass the seeds directly to the ST Engine
    MotionGC_SetCalParams(&initial_bias);
}

bool write_gyro_bias_to_flash(MGC_output_t* new_bias) {
    calibration_registry_t data_to_write;

    data_to_write.magic = FLASH_MAGIC_NUMBER;
    data_to_write.gyro_bias_x = new_bias->GyroBiasX;
    data_to_write.gyro_bias_y = new_bias->GyroBiasY;
    data_to_write.gyro_bias_z = new_bias->GyroBiasZ;
    data_to_write.checksum = (uint32_t)(new_bias->GyroBiasX * 100) +
                             (uint32_t)(new_bias->GyroBiasY * 100) +
                             (uint32_t)(new_bias->GyroBiasZ * 100);

    // Prepare a 32-byte (256-bit) flash word buffer aligned in RAM
    uint32_t flash_word_buffer[8] = {0};
    memcpy(flash_word_buffer, &data_to_write, sizeof(calibration_registry_t));

    // 1. Unlock Flash
    HAL_FLASH_Unlock();

    // 2. Erase the specific Sector
    FLASH_EraseInitTypeDef erase_config;
    uint32_t sector_error = 0;

    erase_config.TypeErase   = FLASH_TYPEERASE_SECTORS;
    erase_config.Banks       = TARGET_FLASH_BANK;
    erase_config.Sector      = TARGET_FLASH_SECTOR;
    erase_config.NbSectors   = 1;
    erase_config.VoltageRange = FLASH_VOLTAGE_RANGE_3; // Standard 3.3V operation

    // Note: Clear flash flags prior to programming to flush any previous errors
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);

    if (HAL_FLASHEx_Erase(&erase_config, &sector_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    // 3. Program the 256-bit Flash Word
    // On the H7, the third parameter MUST be the memory pointer to your 32-byte source array cast to a uint32_t
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, CALIB_FLASH_ADDR, (uint32_t)flash_word_buffer) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    // 4. Re-lock Flash
    HAL_FLASH_Lock();
    return true;
}

void process_motion_gc_autosave(MGC_output_t* current_bias, int bias_updated) {
    static MGC_output_t last_saved_bias = {0.0f, 0.0f, 0.0f};
    static bool initialization_snapshot = false;

    // Cache the initial flash values on boot so we don't overwrite them immediately
    if (!initialization_snapshot) {
        calibration_registry_t flash_data;
        memcpy(&flash_data, (uint32_t*)CALIB_FLASH_ADDR, sizeof(calibration_registry_t));
        if (flash_data.magic == FLASH_MAGIC_NUMBER) {
            last_saved_bias.GyroBiasX = flash_data.gyro_bias_x;
            last_saved_bias.GyroBiasY = flash_data.gyro_bias_y;
            last_saved_bias.GyroBiasZ = flash_data.gyro_bias_z;
        }
        initialization_snapshot = true;
    }

    // Check conditions: Only update if background calibration completed a refinement step while disarmed
    if (bias_updated ) {

        // Calculate the absolute differences to track physical thermal shift
        float delta_x = fabsf(current_bias->GyroBiasX - last_saved_bias.GyroBiasX);
        float delta_y = fabsf(current_bias->GyroBiasY - last_saved_bias.GyroBiasY);
        float delta_z = fabsf(current_bias->GyroBiasZ - last_saved_bias.GyroBiasZ);

        // Save only if drift exceeds 0.05 degrees per second on any axis
        if (delta_x > 0.05f || delta_y > 0.05f || delta_z > 0.05f) {
            if (write_gyro_bias_to_flash(current_bias)) {
                // Keep local copy up to date if write succeeded
                last_saved_bias = *current_bias;
            }
        }
    }
}
