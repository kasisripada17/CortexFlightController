/*
 * sensor_fusion.h
 *
 *  Created on: 18-Apr-2026
 *      Author: kasiviswanadhsripada
 */

#ifndef INC_GYRO_CALIBRATION_H_
#define INC_GYRO_CALIBRATION_H_

#include <math.h>
#include "stm32h7xx_hal.h"
#include "motion_gc.h"
#include <stdbool.h>

#define CALIB_FLASH_ADDR    0x080E0000  // Must be 32-byte aligned! (e.g., ends in 00, 20, 40, 60...)
#define TARGET_FLASH_SECTOR FLASH_SECTOR_7
#define TARGET_FLASH_BANK   FLASH_BANK_1
#define FLASH_MAGIC_NUMBER  0x4D474331  // "MGC1" in ASCII hex representation
typedef struct {
    uint32_t magic;         // Validation token
    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;
    uint32_t checksum;
    // The struct is 20 bytes. H7 requires a 32-byte chunk,
    // so the rest will be implicitly padded with zeros when we transfer it to the flash buffer.
} calibration_registry_t;


void gyro_calibration_init(void) ;
void gyro_calibration_routine(void);
void process_motion_gc_autosave(MGC_output_t* current_bias, int bias_updated);
bool write_gyro_bias_to_flash(MGC_output_t* new_bias);
void load_gyro_seed_from_flash(void);
// Constants
#define DEG_TO_RAD 0.01745329251f
#define G_VALUE 1.0f  // Use 1.0 if raw_acc is in Gs, or 9.81 if in m/s^2




#endif /* INC_GYRO_CALIBRATION_H_ */
