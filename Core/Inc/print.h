/*
 * print.h
 *
 *  Created on: 08-Apr-2026
 *      Author: kasiviswanadhsripada
 */

#ifndef INC_PRINT_H_
#define INC_PRINT_H_


#include <stdint.h>
#include "usbd_cdc_if.h"
#include "usb_device.h"
void usb_print(uint8_t *buffer, uint16_t size);
void MS5611_Init(void) ;

void Calibrate_Baro() ;

#endif /* INC_PRINT_H_ */
