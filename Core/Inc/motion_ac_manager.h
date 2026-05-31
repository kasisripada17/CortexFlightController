/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MOTION_AC_MANAGER_H
#define MOTION_AC_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "motion_ac.h"
#include "main.h"

/* Extern variables ----------------------------------------------------------*/
/* Exported Macros -----------------------------------------------------------*/
/* Exported Types ------------------------------------------------------------*/
typedef enum {
	DYNAMIC_CALIBRATION = 0, SIX_POINT_CALIBRATION = 1
} MAC_calibration_mode_t;

typedef enum {
	MAC_DISABLE_LIB = 0, MAC_ENABLE_LIB = 1
} MAC_enable_lib_t;

typedef struct {
	float x, y, z;
} MOTION_SENSOR_Axes_t;

/* Imported Variables --------------------------------------------------------*/
/* Exported Functions Prototypes ---------------------------------------------*/
void MotionAC_manager_init(MAC_enable_lib_t enable);
void MotionAC_manager_update(MAC_input_t *data_in, uint8_t *is_calibrated);
void MotionAC_manager_get_params(MAC_output_t *data_out);
void MotionAC_manager_get_version(char *version, int32_t *length);
void MotionAC_manager_compensate(MOTION_SENSOR_Axes_t *DataIn, MOTION_SENSOR_Axes_t *DataOut);

int16_t acc_bias_to_mg(float acc_bias);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_AC_MANAGER_H */
