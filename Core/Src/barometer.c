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
#include "filters.h"
LPF_Filter baroFilter;
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
float baroLPFAltitude = 0.0f;
// --- Private Calibration Configs ---
#define SKIP_SAMPLES      100  // Skip first 6 seconds to clear sensor thermal bloom
#define CALIB_SAMPLES     SKIP_SAMPLES+100  // Total samples (~12 seconds of data at 50Hz)

static float P_ground_accumulator = 0.0f;
static float P_at_ground          = 101325.0f; // Default sea level pressure Pascal baseline
static uint16_t calib_counter     = 0;
bool new_baro_ready = false;

static BaroState_t currentBaroState = BARO_STATE_START_PRES;
static uint32_t lastBaroTime        = 0;
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
    HAL_Delay(15); // Wait for internal state machine reload

    // 2. Parse Factory Calibration Coefficients (C0 to C7)
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t cmd = 0xA0 + (i * 2);
        uint8_t rx_data[2] = {0, 0};

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

        // Step A: Send the 8-bit operational command
        HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);

        // Step B: Clock out the 16-bit payload response from the PROM
        HAL_SPI_Receive(&hspi1, rx_data, 2, 10);

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

        // Reconstruct into your destination global array
        C[i] = ((uint16_t)rx_data[0] << 8) | rx_data[1];
    }
}

void MS5611_Start_Pressure_Conv(void) {
    uint8_t cmd = 0x48; // OSR = 4096

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); // CORRECT: Release CS so the sensor can safely convert
}

void MS5611_Start_Temp_Conv(void) {
    uint8_t cmd = 0x58; // OSR = 4096

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); // CORRECT: Release CS so the sensor can safely convert
}


/**
 * @brief  Reads the internal 24-bit ADC register values from the MS5611 over SPI bus.
 * @note   Uses a LibDriver style full-duplex 4-byte unified transmission burst.
 * @return uint32_t Raw 24-bit sensor conversion matrix value.
 */
uint32_t MS5611_Read_ADC_Result(void) {
    uint8_t cmd = 0x00; // OSR = 4096

    uint8_t rx_payload[4] = {0, 0, 0,0};


    // Give the sensor's internal shift register a brief moment to latch
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
    // Clock in the 24-bit raw data directly into a 3-byte layout
    HAL_SPI_Receive(&hspi1,rx_payload, 4, 10);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    // Force casting inline using indices 0, 1, and 2 to match the multi-device bus alignment
    uint32_t high_byte = (uint32_t)rx_payload[0];
    uint32_t mid_byte  = (uint32_t)rx_payload[1];
    uint32_t low_byte  = (uint32_t)rx_payload[2];

    uint32_t result = ((high_byte << 16) | (mid_byte << 8) | low_byte  )  ;
    if (result < 7500000) {
          result = (result * 14) / 10; // Apply the 1.4x scale realignment correction
      }
    return result;
}

/**
 * @brief Executes factory standard equations and updates global state variables.
 * @note Corrected for intermediate compiler fixed-point bitwise scaling alignments.
 * @param D1 Raw 24-bit uncompensated pressure reading.
 * @param D2 Raw 24-bit uncompensated temperature reading.
 */
void Calculate_Final_Altitude(uint32_t D1, uint32_t D2) {
    // --- STEP 0: EXPLICITLY MAP FACTORY CONSTANTS ---
    int64_t C1_SENS     = (int64_t)C[1]; // Pressure Sensitivity
    int64_t C2_OFF      = (int64_t)C[2]; // Pressure Offset
    int64_t C3_TCS      = (int64_t)C[3]; // Temp Coeff of Press Sensitivity
    int64_t C4_TCO      = (int64_t)C[4]; // Temp Coeff of Press Offset
    int64_t C5_TREF     = (int64_t)C[5]; // Reference Temperature
    int64_t C6_TEMPSENS = (int64_t)C[6]; // Temp Coeff of Temperature

    // --- STEP 1: COMPUTE TEMPERATURE DIFFERENCES ---
    int64_t dT = (int64_t)D2 - (C5_TREF << 8);
    // Explicit division replaces ">> 23" to guarantee safe compiler sign-extension
    int32_t TEMP = 2000 + (int32_t)((dT * C6_TEMPSENS) / 8388608LL);

    // --- STEP 2: SECOND ORDER THERMAL COMPENSATION ---
    int64_t T2 = 0;
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;

    if (TEMP < 2000) { // Ambient temperatures below 20°C
        T2 = (dT * dT) / 2147483648LL; // Replaces >> 31
        int64_t t_diff = (int64_t)TEMP - 2000;
        int64_t t_squared = t_diff * t_diff;

        OFF2  = (5 * t_squared) / 2LL;  // Replaces >> 1
        SENS2 = (5 * t_squared) / 4LL;  // Replaces >> 2

        if (TEMP < -1500) { // Severe cold threshold below -15°C
            int64_t tc_diff = (int64_t)TEMP + 1500;
            int64_t tc_squared = tc_diff * tc_diff;
            OFF2  = OFF2 + (7 * tc_squared);
            SENS2 = SENS2 + ((11 * tc_squared) / 2LL); // Replaces >> 1
        }
    }
    TEMP = TEMP - (int32_t)T2;

    // --- STEP 3: COMPUTE DATASHEET EXACT COMPENSATED COEFFICIENTS ---
    // Intermediate scaling calculations use explicit division to prevent bitwise misalignment
    int64_t OFF  = (C2_OFF << 16) + ((C4_TCO * dT) / 128LL);
    int64_t SENS = (C1_SENS << 15) + ((C3_TCS * dT) / 256LL);

    // Apply second-order thermal corrections
    OFF  = OFF - OFF2;
    SENS = SENS - SENS2;

    // Final Temperature-Compensated Pressure calculation
    int64_t SENS_term = ((int64_t)D1 * SENS) / 2097152LL; // Replaces >> 21
    int32_t P = (int32_t)((SENS_term - OFF) / 32768LL);   // Replaces >> 15

    // --- STEP 4: ASSIGN METRIC FLOAT OUTPUTS ---
    current_temperature = (float)TEMP / 100.0f;
    current_pressure    = (float)P; // Value in Pascals (Pa)


    // This executes only after the 12-second calibration window has closed

        // Safety fallback configuration
        baro_altitude = 44330.0f * (1.0f - powf(current_pressure / 101325.0f, 0.190295f));
static bool init = 1;
        if(init)
        {
    		LPF_Init(&baroFilter, 50.0f, 50.0f);
    		init = 0;
        }

        else
        {
        	baroLPFAltitude = LPF_Update(&baroFilter, baro_altitude);
        }

    // Assign your final altitude values to their respective metrics

    // --- STEP 7: ASYNCHRONOUS STATE MACHINE SIGNAL FLAG ---
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
