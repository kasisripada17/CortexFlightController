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



#define GYRO_SW_LPF
#define ACC_SW_LPF



void LPF_Init(LPF_Filter *filter, float cutoff, float sampleRate);
float LPF_Update(LPF_Filter *filter, float input);





#endif /* INC_FILTERS_H_ */
