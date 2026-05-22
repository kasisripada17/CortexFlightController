/*
 * telemetry.h
 *
 *  Created on: 07-May-2026
 *      Author: kasiviswanadhsripada
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

#include <stdint.h>

typedef struct {
    uint8_t  header;    // Always 0x10 for Data Frame
    uint16_t sensor_id; // e.g., 0x0100 for Altitude
    int32_t  value;     // The actual data
    uint8_t  checksum;
} FrSky_Frame_t;

void Send_S_Port_Frame_Fast(uint16_t id, int32_t val) ;



#endif /* INC_TELEMETRY_H_ */
