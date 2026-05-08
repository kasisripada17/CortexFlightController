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
extern uint8_t buffer[256];



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
    // 1. Determine which axis we are tuning
	// Switch positions: Low = Roll, Mid = Pitch, High = Yaw
	if (radio.pid_channel_selector < 1300)
		PID_tuning_channel = PID_ROLL;	// Roll
	else if (radio.pid_channel_selector < 1700)
		PID_tuning_channel = PID_PITCH;	// Roll
	else
		PID_tuning_channel = PID_YAW;	// Roll

	// 2. Determine which PID term we are tuning
	// Switch positions: Low = P, Mid = I, High = D
	if (radio.pid_tune_gain_selector < 1300)
		PID_tuning_gain = PID_KP;      // P
	else if (radio.pid_tune_gain_selector< 1700)
		PID_tuning_gain = PID_KI;
	else
		PID_tuning_gain = PID_KD;

	float knob_percent = (float)(radio.pid_gain - 1000) / 1000.0f;
	if (knob_percent < 0) knob_percent = 0;
	if (knob_percent > 1) knob_percent = 1;

	float new_gain = 0.0f;

	// 4. Apply scale based on which term is selected
	switch (PID_tuning_gain) {
	    case PID_KP:
	        new_gain = knob_percent * 10.0f;  // Range: 0.0 to 10.0
	        break;
	    case PID_KI:
	        new_gain = knob_percent * 2.0f;   // Range: 0.0 to 2.0
	        break;
	    case PID_KD:
	        new_gain = knob_percent * 0.1f;   // Range: 0.0 to 0.1
	        break;
	}

	// 5. Inject into the selected Axis
    PID_Controller* target_pid = NULL;
	if (PID_tuning_channel == PID_ROLL)
		target_pid = &fc.roll;
	else if (PID_tuning_channel == PID_PITCH)
		target_pid = &fc.pitch;
	else
		target_pid = &fc.yaw;

	// 6. Assignment
	if (PID_tuning_gain == PID_KP)
		target_pid->kp = new_gain;
	else if (PID_tuning_gain == PID_KI)
		target_pid->ki = new_gain;
	else if (PID_tuning_gain == PID_KD)
		target_pid->kd = new_gain;
	uint8_t size = sprintf(buffer, "%d, %d, %f\r\n",PID_tuning_channel,PID_tuning_gain,new_gain);
	usb_print(buffer,size);
}
