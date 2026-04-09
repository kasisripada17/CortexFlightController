
#include "radio.h"

uint32_t IC_Val1 = 0, IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured[4] = {0,0,0,0};  // Flags for 4 channels
uint32_t receiver[4] = {0,0,0,0};      // Resulting 1000-2000us values

receiver_t radio;

float normalize_radio(uint16_t pwm_val) {
    // 1. Center the value around 0 (Result: -500 to 500)
    float centered = (float)pwm_val - 1500.0f;

    // 2. Add a small deadband (e.g., +/- 10us) to ignore jittery gimbals
    if (fabsf(centered) < 10.0f) return 0.0f;

    // 3. Scale to -1.0 to 1.0 range
    return centered / 500.0f;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        uint32_t channel_index = 0;
        uint32_t active_channel = 0;

        // Identify which channel triggered the interrupt
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) { channel_index = 0; active_channel = TIM_CHANNEL_1; }
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) { channel_index = 1; active_channel = TIM_CHANNEL_2; }
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) { channel_index = 2; active_channel = TIM_CHANNEL_3; }
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) { channel_index = 3; active_channel = TIM_CHANNEL_4; }

        if (Is_First_Captured[channel_index] == 0) // First edge (Rising)
        {
            IC_Val1 = HAL_TIM_ReadCapturedValue(htim, active_channel);
            Is_First_Captured[channel_index] = 1;

            // Switch polarity to look for the Falling edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, active_channel, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else // Second edge (Falling)
        {
            IC_Val2 = HAL_TIM_ReadCapturedValue(htim, active_channel);

            if (IC_Val2 > IC_Val1)
            {
            	receiver[channel_index] = IC_Val2 - IC_Val1;
            }
            else if (IC_Val1 > IC_Val2) // Handle Timer Overflow
            {
            	receiver[channel_index] = (0xFFFF - IC_Val1) + IC_Val2;
            }

            Is_First_Captured[channel_index] = 0;

            // Reset polarity to Rising edge for next pulse
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, active_channel, TIM_INPUTCHANNELPOLARITY_RISING);
        }
        radio.throttle = receiver[0];
        radio.roll = receiver[1];
        radio.pitch = receiver[2];
        radio.yaw = receiver[3];

    }
}



