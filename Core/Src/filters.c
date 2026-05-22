/*
 * filters.c
 *
 *  Created on: 10-May-2026
 *      Author: kasiviswanadhsripada
 */

#include"filters.h"



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
