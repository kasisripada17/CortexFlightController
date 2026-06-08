/*
 * print.c
 *
 *  Created on: 08-Apr-2026
 *      Author: kasiviswanadhsripada
 */

#include "print.h"

//#define DEBUG_BUILD

extern USBD_HandleTypeDef hUsbDeviceHS;
extern uint8_t UserRxBufferHS[APP_RX_DATA_SIZE];
void usb_print(char *buffer, uint16_t size)
{

#ifdef DEBUG_BUILD
	CDC_Transmit_HS((uint8_t*)buffer,size);
#endif
}


