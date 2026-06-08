#include "main.h"
#include <stdlib.h>
#include <string.h>
#include "pid_control.h"
#include <stdio.h>
#include "usbd_cdc_if.h"
#include  "radio.h"
#include "print.h"

typedef enum {
    COMMS_STATE_IDLE,
    COMMS_STATE_PAYLOAD
} CommsState_t;


typedef enum {
	PID_ROLL,
	PID_PITCH,
	PID_YAW,
	PID_KP,
	PID_KI,
	PID_KD
}PID_tuning_t;
PID_tuning_t PID_tuning_channel = PID_ROLL;
PID_tuning_t PID_tuning_gain = PID_KP;
extern char buffer[256];



static char rx_buf[32];
static uint8_t rx_idx = 0;
static CommsState_t state = COMMS_STATE_IDLE;
extern Flight_Control_t fc; // The global flight control object
extern volatile receiver_t radio;
// Forward declaration
void Comms_ApplyLogic(void);

void Comms_ProcessIncoming(uint8_t* Buf, uint32_t Len) {
    for (uint32_t i = 0; i < Len; i++) {
        uint8_t c = Buf[i];

        switch (state) {
            case COMMS_STATE_IDLE:
                if (c == '$') {
                    rx_idx = 0;
                    state = COMMS_STATE_PAYLOAD;
                }
                break;

            case COMMS_STATE_PAYLOAD:
                if (c == '\n' || c == '\r') {
                    rx_buf[rx_idx] = '\0';
                    Comms_ApplyLogic();
                    state = COMMS_STATE_IDLE;
                } else if (rx_idx < 31) {
                    rx_buf[rx_idx++] = c;
                } else {
                    state = COMMS_STATE_IDLE; // Safety reset
                }
                break;
        }
    }
}
void Comms_ApplyLogic(void) {
    if (strlen(rx_buf) < 3) return;

    char axis  = rx_buf[0];
    char param = rx_buf[1];
    float val  = atof(&rx_buf[2]);

    PID_Controller* target = NULL;

    if (axis == 'P')
    	target = &fc.pitch;
    else if (axis == 'R')
    	target = &fc.roll;
    else if (axis == 'Y')
    	target = &fc.yaw;

    if (target != NULL) {
        if (param == 'p')
        	target->kp = val;
        else if (param == 'i')
        	target->ki = val;
        else if (param == 'd')
        	target->kd = val;

        // --- VERIFICATION STEP ---
        char tx_msg[64];
        // Format: ACK [Axis] [Param] = [Value]
        int len = sprintf(tx_msg, "ACK %c%c set to %.4f\r\n", axis, param, val);

        // Transmit back to Python/Terminal
        CDC_Transmit_HS((uint8_t*)tx_msg, len);

        // Reset state to keep the flight stable
        target->integral = 0;
        target->last_input = 0;
    }
}
//radio.pid_channel_selector = pwm_channels[5];
//   radio.pid_tune_gain_selector = pwm_channels[6
void update_tuning_from_radio(void) {



	float p_gain = (float)(radio.p_gain - 1000) / 1000.0f;
	float i_gain = (float)(radio.i_gain - 1000) / 1000.0f;
	float d_gain = (float)(radio.d_gain - 1000) / 1000.0f;

	if (p_gain < 0.0f) p_gain = 0.0f;
	if (p_gain > 1.0f) p_gain = 1.0f;
	if (i_gain < 0.0f) i_gain = 0.0f;
	if (i_gain > 1.0f) i_gain = 1.0f;
	if (d_gain < 0.0f) d_gain = 0.0f;
	if (d_gain > 1.0f) d_gain = 1.0f;



	p_gain = p_gain * 500.0f;  // Range: 0.0 to 10.0

	i_gain = i_gain * 50.0f;   // Range: 0.0 to 2.0

	d_gain = d_gain * 5.0f;   // Range: 0.0 to 0.1

//	fc.roll.kp = p_gain;
//	fc.pitch.kp = p_gain;
//
//	fc.roll.ki = i_gain;
//	fc.pitch.ki = i_gain;
//
//	fc.roll.kd = d_gain;
//	fc.pitch.kd = d_gain;


		fc.yaw.kp = p_gain;

		fc.yaw.ki = i_gain;
	//

}
