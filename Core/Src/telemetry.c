/*
 * telemetry.c
 *
 *  Created on: 07-May-2026
 *      Author: kasiviswanadhsripada
 */
#include "telemetry.h"
#include "stm32h7xx_hal.h"
#include "main.h"
extern UART_HandleTypeDef huart5;

extern float alt_fused;
extern uint8_t rx_byte;
uint8_t telemetry_request_pending = 0;
uint8_t FrSky_CalculateChecksum(uint8_t *data) ;
void SPort_Write_Byte(uint8_t b) ;

uint8_t FrSky_CalculateChecksum(uint8_t *data) {
    uint32_t sum = 0;
    for (int i = 0; i < 7; i++) {
        sum += data[i];
    }
    // Standard S.Port checksum: sum high byte into low byte
    while (sum > 0xFF) {
        sum = (sum & 0xFF) + (sum >> 8);
    }
    return 0xFF - (uint8_t)sum;
}

void Send_S_Port_Frame_Fast(uint16_t id, int32_t val) {
    uint8_t frame[8] = {0,0,0,0,0,0,0,0};
    frame[0] = 0x10; // Data Header
    frame[1] = id & 0xFF;
    frame[2] = (id >> 8) & 0xFF;
    frame[3] = val & 0xFF;
    frame[4] = (val >> 8) & 0xFF;
    frame[5] = (val >> 16) & 0xFF;
    frame[6] = (val >> 24) & 0xFF;
    frame[7] = FrSky_CalculateChecksum(frame);
    // CRITICAL: Disable Receiver immediately to stop self-echo
    UART5->CR1 &= ~USART_CR1_RE;

    // 1. Send Header (NEVER STUFFED)
    HAL_UART_Transmit(&huart5,frame, 8, 1);


       // Clear the RDR (Receive Data Register) to dump our own echo
       (void)UART5->RDR;

       // Clear all status flags
       UART5->ICR |= 0x123BFF;

       // Re-enable Receiver
       UART5->CR1 |= USART_CR1_RE;
}



	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	    if (huart->Instance == UART5) {
	        // 1. Immediately Clear all Hardware flags to prevent lock-up
	        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);

	        if (rx_byte == 0x1B) {
	            // 2. Short turnaround delay (~150us)
	            for(volatile uint32_t i = 0; i < 60000; i++);

	            // 3. Send Data
	            Send_S_Port_Frame_Fast(0x0100, (int32_t)(1260));

	            // 4. CRITICAL: Dump the RDR to clear the "echo" of our own transmission
	            // This prevents the next interrupt from being triggered by our own data
	            (void)UART5->RDR;
	        }

	        // 5. Re-prime the interrupt
	        HAL_UART_Receive_IT(&huart5, &rx_byte, 1);
	    }
	}
