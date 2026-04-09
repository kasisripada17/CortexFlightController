/*
 * print.c
 *
 *  Created on: 08-Apr-2026
 *      Author: kasiviswanadhsripada
 */

#include "print.h"

void usb_print(uint8_t *buffer, uint16_t size)
{
	CDC_Transmit_HS(buffer,size);
}
