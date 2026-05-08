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


// Standard FrSky Checksum: Fold carries back into the byte
uint8_t FrSky_CalculateChecksum(uint8_t *data) {
    uint32_t sum = 0;
    for (int i = 0; i < 7; i++) {
        sum += data[i];
        if (sum > 0xFF) sum = (sum & 0xFF) + 1;
    }
    return 0xFF - (uint8_t)(sum & 0xFF);
}

// Byte Stuffing: Only used for Data and Checksum, NOT the 0x10 Header
void SPort_Write_Byte_Stuffed(uint8_t b) {
    if (b == 0x7E || b == 0x7D) {
        uint8_t stuffed[2];
        stuffed[0] = 0x7D;
        stuffed[1] = b ^ 0x20;
        HAL_UART_Transmit(&huart5, stuffed, 2, 5); // Short timeout
    } else {
        HAL_UART_Transmit(&huart5, &b, 1, 5);
    }
}

void Send_S_Port_Frame(uint16_t id, int32_t val) {
    uint8_t frame[7];
    frame[0] = 0x10; // Data Frame Header
    frame[1] = id & 0xFF;
    frame[2] = (id >> 8) & 0xFF;
    frame[3] = val & 0xFF;
    frame[4] = (val >> 8) & 0xFF;
    frame[5] = (val >> 16) & 0xFF;
    frame[6] = (val >> 24) & 0xFF;

    uint8_t checksum = FrSky_CalculateChecksum(frame);

    // --- HALF DUPLEX CRITICAL SECTION ---
    // 1. Disable Receiver so we don't trigger an interrupt on our own TX
    CLEAR_BIT(huart5.Instance->CR1, USART_CR1_RE);

    // 2. Send 0x10 Header (NEVER stuffed)
    uint8_t header = 0x10;
    HAL_UART_Transmit(&huart5,frame, 8, 1);


    // 4. Re-enable Receiver for the next poll
    SET_BIT(huart5.Instance->CR1, USART_CR1_RE);
    // --- END CRITICAL SECTION ---
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART5) {
        if (rx_byte == 0x1B) { // XSR Physical ID
            static int32_t test_val = 0;

            // Immediately respond
            Send_S_Port_Frame(0x0100, test_val++);
        }
        // Priming the next receive
        HAL_UART_Receive_IT(&huart5, &rx_byte, 1);
    }
}
void SPort_Write_Byte(uint8_t b) {
    if (b == 0x7E || b == 0x7D) {
        uint8_t stuffed[2];
        stuffed[0] = 0x7D;     // The escape character
        stuffed[1] = b ^ 0x20; // The XORed data
        HAL_UART_Transmit(&huart5, stuffed, 2, 1);
    } else {
        HAL_UART_Transmit(&huart5, &b, 1, 1);
    }
}
