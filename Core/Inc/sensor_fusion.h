/*
 * sensor_fusion.h
 *
 *  Created on: 18-Apr-2026
 *      Author: kasiviswanadhsripada
 */

#ifndef INC_SENSOR_FUSION_H_
#define INC_SENSOR_FUSION_H_

#include <math.h>

// Constants
#define DEG_TO_RAD 0.01745329251f
#define G_VALUE 1.0f  // Use 1.0 if raw_acc is in Gs, or 9.81 if in m/s^2

typedef struct {
    float x, y, z;
} Vector3f;


#endif /* INC_SENSOR_FUSION_H_ */
