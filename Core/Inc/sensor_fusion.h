/*
 * sensor_fusion.h
 *
 *  Created on: 20-Apr-2026
 *      Author: kasiviswanadhsripada
 */

#ifndef INC_SENSOR_FUSION_H_
#define INC_SENSOR_FUSION_H_


typedef struct {
    float x;
    float y;
    float z;
} Vector3f_t;
void sensor_fusion_init(void);

void sensor_fusion_update(void);



#endif /* INC_SENSOR_FUSION_H_ */
