/*
 * sensor_fusion.c
 *
 *  Created on: 18-Apr-2026
 *      Author: kasiviswanadhsripada
 */
#include "sensor_fusion.h"
/**
 * @brief Removes gravity vector from raw accelerometer data to get linear acceleration.
 * @param raw_acc: The raw accelerometer values (X, Y, Z)
 * @param roll: Current drone roll in degrees
 * @param pitch: Current drone pitch in degrees
 * @return Vector3f: The linear acceleration (movement only)
 */
Vector3f get_linear_acceleration(Vector3f raw_acc, float roll_deg, float pitch_deg) {
    Vector3f linear_acc;

    // 1. Convert degrees to radians
    float roll_rad = roll_deg * DEG_TO_RAD;
    float pitch_rad = pitch_deg * DEG_TO_RAD;

    // 2. Pre-calculate trig values for efficiency
    float cosRoll  = cosf(roll_rad);
    float sinRoll  = sinf(roll_rad);
    float cosPitch = cosf(pitch_rad);
    float sinPitch = sinf(pitch_rad);

    // 3. Subtract gravity vector component from each axis
    // Acc_linear_x = Acc_raw_x - sin(Pitch)
    linear_acc.x = raw_acc.x - (sinPitch * G_VALUE);

    // Acc_linear_y = Acc_raw_y - sin(Roll) * cos(Pitch)
    linear_acc.y = raw_acc.y - (sinRoll * cosPitch * G_VALUE);

    // Acc_linear_z = Acc_raw_z - cos(Roll) * cos(Pitch)
    linear_acc.z = raw_acc.z - (cosRoll * cosPitch * G_VALUE);

    return linear_acc;
}

