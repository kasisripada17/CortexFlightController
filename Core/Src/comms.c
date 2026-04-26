#include "main.h"
#include <stdlib.h>
#include <string.h>
#include "pid_control.h"
#include <stdio.h>
#include "usbd_cdc_if.h"

typedef enum {
    COMMS_STATE_IDLE,
    COMMS_STATE_PAYLOAD
} CommsState_t;

static char rx_buf[32];
static uint8_t rx_idx = 0;
static CommsState_t state = COMMS_STATE_IDLE;
extern Flight_Control_t fc; // The global flight control object
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
