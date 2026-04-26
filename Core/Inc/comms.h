/*
 * comms.h
 *
 *  Created on: 26-Apr-2026
 *      Author: kasiviswanadhsripada
 */

#ifndef INC_COMMS_H_
#define INC_COMMS_H_

#include "flight_control.h"
#include "motors.h"
#include "radio.h"
#include "lsm6ds3.h"
#include "pid_control.h"
#include <stdbool.h>
#include "print.h"

typedef enum {
    WAIT_START,
    RECEIVING_PAYLOAD
} RX_State;
void process_robust_usb(uint8_t* Buf, uint32_t Len) ;
void execute_pid_update(char* cmd) ;


#endif /* INC_COMMS_H_ */
