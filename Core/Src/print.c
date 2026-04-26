/*
 * print.c
 *
 *  Created on: 08-Apr-2026
 *      Author: kasiviswanadhsripada
 */

#include "print.h"
extern USBD_HandleTypeDef hUsbDeviceHS;
extern uint8_t UserRxBufferHS[APP_RX_DATA_SIZE];
void usb_print(uint8_t *buffer, uint16_t size)
{
	CDC_Transmit_HS(buffer,size);
	static int a = 0;


}


