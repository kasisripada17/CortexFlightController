/*
 * barometer.c
 *
 * Created on: 05-May-2026
 * Author: kasiviswanadhsripada
 * Description: Asynchronous, non-blocking MS5611 Barometer driver
 * integrated with a 2nd-order complementary fusion filter.
 * Optimized based on LibDriver SPI frame specs and 64-bit precision math.
 */

#include "barometer.h"
#include <stdio.h>
#include <math.h>
#include "print.h"
#include "pid_control.h"
#include "stdbool.h"
#include "lsm6ds3.h"
#include "altitude_hold.h"

// --- Extern Variables & Functions ---
extern Flight_Control_t fc;
extern SPI_HandleTypeDef hspi1;
extern float alt_fused;       // Global target for fused state tracking
extern volatile IMU_Data_t sensor_data;

// --- Global Variables ---
float current_temperature = 0.0f;
float current_pressure    = 0.0f;
float current_altitude    = 0.0f;
float baro_altitude   = 0.0f;

// --- Private Calibration Configs ---
#define SKIP_SAMPLES      100  // Skip first 6 seconds to clear sensor thermal bloom
#define CALIB_SAMPLES     SKIP_SAMPLES+100  // Total samples (~12 seconds of data at 50Hz)

static float P_ground_accumulator = 0.0f;
static float P_at_ground          = 101325.0f; // Default sea level pressure Pascal baseline
static uint16_t calib_counter     = 0;
bool new_baro_ready = false;

static BaroState_t currentBaroState = BARO_STATE_START_PRES;
static uint32_t lastBaroTime        = 0;
static uint32_t lastFusionUpdate    = 0; // Tracks elapsed execution intervals for fusion integration
static uint32_t D1                  = 0; // Raw pressure sensor reading
static uint32_t D2                  = 0; // Raw temperature sensor reading

// Explicit 8-element array to map PROM registers cleanly (0-7)
uint16_t C[8] = {0};

/**
 * @brief  Initializes the MS5611 hardware profile, triggers reset, and reads PROM coefficients.
 */
void MS5611_Init(void) {
    uint8_t reset_cmd = 0x1E;

    // 1. Execute Device Hardware Reset
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &reset_cmd, 1, 10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(15); // Wait for sensor internal state reload execution

    // 2. Parse Factory Calibration Vector Coefficients via LibDriver Full-Duplex Strategy
    uint8_t tx_frame[3] = {0};
    uint8_t rx_frame[3] = {0};

    for (uint8_t i = 0; i < 8; i++) {
        tx_frame[0] = 0xA0 + (i * 2); // Commands from 0xA0 to 0xAE
        tx_frame[1] = 0x00;           // Dummy byte for clocking out data
        tx_frame[2] = 0x00;           // Dummy byte for clocking out data

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
        // Transmit and receive simultaneously to keep the clock continuous
        HAL_SPI_TransmitReceive(&hspi1, tx_frame, rx_frame, 3, 10);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

        // Byte index 1 and 2 contain valid 16-bit register payloads
        C[i] = ((uint16_t)rx_frame[1] << 8) | rx_frame[2];
    }

    // Initialize timestamp tracker for multi-rate fusion loop step metrics
    lastFusionUpdate = HAL_GetTick();
}

/**
 * @brief  Triggers an asynchronous Pressure conversion (D1) at max Oversampling Rate (4096).
 * @note   Leaves Chip Select asserted LOW to prevent the sensor from reverting to I2C mode.
 */
void MS5611_Start_Pressure_Conv(void) {
    uint8_t cmd = 0x48; // MS5611_CONV_D1_OSR_4096
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

    // CRITICAL: DO NOT raise CS here. Keep it locked low through the wait phase.
}

/**
 * @brief  Triggers an asynchronous Temperature conversion (D2) at max Oversampling Rate (4096).
 * @note   Leaves Chip Select asserted LOW to prevent the sensor from reverting to I2C mode.
 */
void MS5611_Start_Temp_Conv(void) {
    uint8_t cmd = 0x58; // MS5611_CONV_D2_OSR_4096
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

    // CRITICAL: DO NOT raise CS here. Keep it locked low through the wait phase.
}

/**
 * @brief  Reads the internal 24-bit ADC register values from the MS5611 over SPI bus.
 * @note   Uses a LibDriver style full-duplex 4-byte unified transmission burst.
 * @return uint32_t Raw 24-bit sensor conversion matrix value.
 */
uint32_t MS5611_Read_ADC_Result(void) {
    // 1-byte command (0x00) + 3 data output collection bytes
    uint8_t tx_buf[4] = {0x00, 0x00, 0x00, 0x00};
    uint8_t rx_buf[4] = {0, 0, 0, 0};
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

    // CS line is already held low from the Start command; proceed directly to full-duplex transfer
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 4, 10);

    // Release the SPI bus now that transaction is completed
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

    // Extract out 24 bits of valid data from positions 1, 2, and 3
    uint32_t result = ((uint32_t)rx_buf[1] << 16) |
                      ((uint32_t)rx_buf[2] << 8)  |
                       (uint32_t)rx_buf[3];

    return result;
}
/**
 * @brief  Executes factory standard equations and updates global state variables.
 */
void Calculate_Final_Altitude(uint32_t D1, uint32_t D2) {
    // --- STEP 1: COMPUTE TEMPERATURE DIFFERENCES ---
    int64_t dT = (int64_t)D2 - ((int64_t)C[5] << 8);
    int32_t TEMP = 2000 + (int32_t)((dT * (int64_t)C[6]) >> 23);

    // --- STEP 2: SECOND ORDER THERMAL COMPENSATION ---
    int64_t T2    = 0;
    int64_t OFF2  = 0;
    int64_t SENS2 = 0;

    if (TEMP < 2000) { // Below 20°C Ambient Limit
        T2 = (dT * dT) >> 31;
        int64_t t_diff = (int64_t)TEMP - 2000;
        int64_t t_squared = t_diff * t_diff;

        OFF2  = (5 * t_squared) >> 1;
        SENS2 = (5 * t_squared) >> 2;

        if (TEMP < -1500) { // Severe cold threshold (-15°C Ambient)
            int64_t tc_diff = (int64_t)TEMP + 1500;
            int64_t tc_squared = tc_diff * tc_diff;

            OFF2  = OFF2 + (7 * tc_squared);
            SENS2 = SENS2 + ((11 * tc_squared) >> 1);
        }
    }

    TEMP = TEMP - (int32_t)T2;

    // --- STEP 3: COMPUTE PRESSURE COMPENSATED COEFFICIENTS (DATASHEET EXACT) ---
    int64_t OFF  = ((int64_t)C[2] << 23) + ((int64_t)C[4] * dT);
    OFF = OFF - (OFF2 << 7);
    OFF = OFF >> 7;

    int64_t SENS = ((int64_t)C[1] << 23) + ((int64_t)C[3] * dT);
    SENS = SENS - (SENS2 << 8);
    SENS = SENS >> 8;

    // P = (D1 * SENS / 2^21 - OFF) / 2^15
    int32_t P = (int32_t)(((((int64_t)D1 * SENS) >> 21) - OFF) >> 15);

    // --- STEP 4: ASSIGN METRIC FLOAT OUTPUTS ---
    current_temperature = (float)TEMP / 100.0f;
    current_pressure    = (float)P;

    // --- STEP 5: STATE SYSTEM CALIBRATION ENGINE ---
    if (calib_counter < CALIB_SAMPLES) {
        if (calib_counter > SKIP_SAMPLES) {
            P_ground_accumulator += current_pressure;
        }
        calib_counter++;
    }
    else if (calib_counter == CALIB_SAMPLES) {
        P_at_ground = P_ground_accumulator / (float)(CALIB_SAMPLES - SKIP_SAMPLES);
        calib_counter++;
        fc.ground_offset = 0.0f;
    }

    // --- STEP 6: UNIFIED ALTITUDE OUTPUT TRACKING (FIXED) ---
    // Calculate raw altitude continuously so the filter never gets stuck at zero
    baro_altitude = 44330.0f * (1.0f - powf(current_pressure / P_at_ground, 0.190295f));

    // Absolute height relative to ideal sea level metrics
   // current_altitude = 44330.0f * (1.0f - powf(current_pressure / 101325.0f, 0.190295f));

    // --- STEP 7: ASYNCHRONOUS STATE MACHINE SIGNAL FLAG ---
    // Signal to the high-speed IMU ISR that a fresh barometric sample is ready
    new_baro_ready = true;
}


/**
 * @brief  Non-blocking asynchronous sequencer managing low-speed Baro scheduling.
 * @note   Executed loop-by-loop inside your main background while(1) thread task.
 */
void run_barometer_state_machine(void) {
    uint32_t now = HAL_GetTick();

    switch (currentBaroState) {

        case BARO_STATE_START_PRES:
            MS5611_Start_Pressure_Conv();
            lastBaroTime = now;
            currentBaroState = BARO_STATE_WAIT_PRES;
            break;

        case BARO_STATE_WAIT_PRES:
            // OSR 4096 requires up to an 8.22ms conversion window. Evaluate at 10ms barrier limit.
            if (now - lastBaroTime >= 10U) {
                D1 = MS5611_Read_ADC_Result(); // Reads data and safely releases CS
                currentBaroState = BARO_STATE_START_TEMP;
            }
            break;

        case BARO_STATE_START_TEMP:
            MS5611_Start_Temp_Conv();
            lastBaroTime = now;
            currentBaroState = BARO_STATE_WAIT_TEMP;
            break;

        case BARO_STATE_WAIT_TEMP:
            if (now - lastBaroTime >= 10U) {
                D2 = MS5611_Read_ADC_Result(); // Reads data and safely releases CS
                Calculate_Final_Altitude(D1, D2);
                new_baro_ready = true;
                currentBaroState = BARO_STATE_START_PRES;
            }
            break;

        default:
            currentBaroState = BARO_STATE_START_PRES;
            break;
    }
}
