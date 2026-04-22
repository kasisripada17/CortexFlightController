/*
 * sensor_fusion.h
 *
 *  Created on: 18-Apr-2026
 *      Author: kasiviswanadhsripada
 */

#ifndef INC_GYRO_CALIBRATION_H_
#define INC_GYRO_CALIBRATION_H_

#include <math.h>
void gyro_calibration_init(void) ;

void gyro_calibration_routine(void);

// Constants
#define DEG_TO_RAD 0.01745329251f
#define G_VALUE 1.0f  // Use 1.0 if raw_acc is in Gs, or 9.81 if in m/s^2




#endif /* INC_GYRO_CALIBRATION_H_ */
