
#include "radio.h"
#include "print.h"
#include "motors.h"
uint32_t IC_Val1 = 0, IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured[4] = {0,0,0,0};  // Flags for 4 channels
uint32_t receiver[4] = {0,0,0,0};      // Resulting 1000-2000us values





volatile receiver_t radio;


extern uint8_t sbus_buffer[25] __attribute__((section(".RAM_D2")));
uint16_t sbus_channels[16];
uint8_t failsafe_status = 0;
volatile uint16_t pwm_channels[8];






float normalize_radio(uint16_t pwm_val) {
    // 1. Center the value around 0 (Result: -500 to 500)
    float centered = (float)pwm_val - 1500.0f;

    // 2. Add a small deadband (e.g., +/- 10us) to ignore jittery gimbals
    if (fabsf(centered) < 10.0f) return 0.0f;

    // 3. Scale to -1.0 to 1.0 range
    return centered / 500.0f;
}
#ifdef TRADITIONAL_RECEIVER

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
        radio.throttle = (float)receiver[0];
        radio.roll = (float)receiver[1];
        radio.pitch = (float)receiver[2];
        radio.yaw = (float)receiver[3];


    }
}
#endif
#ifdef SBUS_RECEIVER
void Parse_SBUS() {
	SCB_InvalidateDCache_by_Addr((uint32_t*)sbus_buffer, 25);
    // Check header (0x0F) and footer (0x00) for basic frame validation



	if (sbus_buffer[0] == 0x0F && sbus_buffer[24] == 0x00) {

        // Channel 1 to 4
        sbus_channels[0]  = ((sbus_buffer[1]       | sbus_buffer[2] << 8) & 0x07FF);
        sbus_channels[1]  = ((sbus_buffer[2] >> 3  | sbus_buffer[3] << 5) & 0x07FF);
        sbus_channels[2]  = ((sbus_buffer[3] >> 6  | sbus_buffer[4] << 2 | sbus_buffer[5] << 10) & 0x07FF);
        sbus_channels[3]  = ((sbus_buffer[5] >> 1  | sbus_buffer[6] << 7) & 0x07FF);

        // Channel 5 to 8
        sbus_channels[4]  = ((sbus_buffer[6] >> 4  | sbus_buffer[7] << 4) & 0x07FF);
        sbus_channels[5]  = ((sbus_buffer[7] >> 7  | sbus_buffer[8] << 1 | sbus_buffer[9] << 9) & 0x07FF);
        sbus_channels[6]  = ((sbus_buffer[9] >> 2  | sbus_buffer[10] << 6) & 0x07FF);
        sbus_channels[7]  = ((sbus_buffer[10] >> 5 | sbus_buffer[11] << 3) & 0x07FF);

        // Byte 23 contains Digital Channels (17 & 18), Frame Lost, and Failsafe flags
        // Bit 0: CH17 (digital)
        // Bit 1: CH18 (digital)
        // Bit 2: Frame lost
        // Bit 3: Failsafe activated
        failsafe_status = (sbus_buffer[23] >> 3) & 0x01;
        Update_PWM_Targets();
        radio.roll =  pwm_channels[0];
        radio.pitch =  pwm_channels[1];
        radio.throttle =  pwm_channels[2];
        radio.yaw =  pwm_channels[3];
        radio.mode = pwm_channels[4];

    }
}
uint16_t Map_SBUS_to_PWM(uint16_t sbus_val) {
    // SBUS standard range: 172 to 1811
    // Target PWM range: 1000 to 2000

    // 1. Offset the value to start at 0
    // 2. Scale: (2000-1000) / (1811-172) = 1000 / 1639 ≈ 0.610128
    float pwm = ((float)(sbus_val - 172) * 0.610128f) + 1000.0f;

    // 3. Constrain the output
    if (pwm > 2000) pwm = 2000;
    if (pwm < 1000) pwm = 1000;

    return (uint16_t)pwm;
}


void Update_PWM_Targets() {
    for (int i = 0; i < 8; i++) {
        pwm_channels[i] = Map_SBUS_to_PWM(sbus_channels[i]);
    }
}
#endif
