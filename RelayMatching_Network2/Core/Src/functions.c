/*
 * functions.c
 *
 *  Created on: Jun 1, 2025
 *      Author: Aditya Singh
 *
 *  Refactored for Relay-Based Matching Network Testing
 */
#include "stm32f4xx_hal.h"
#include "main.h"
#include "functions.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "i2ccomm.h"
#include <math.h>
#include <float.h> // For FLT_MAX

// ===================================================================
// 1. CONFIGURATION CONSTANTS
// ===================================================================
#define RELAY_GPIO_PORT     GPIOE
#define RELAY_1PIN          GPIO_PIN_7
#define RELAY_2PIN          GPIO_PIN_8
#define RELAY_3PIN          GPIO_PIN_9
#define RELAY_4PIN          GPIO_PIN_11 // Assuming a 4th relay might exist

// Testing Parameters
#define SETTLE_DELAY_MS     (250)    // Time to wait for circuit to be stable after setting relays
#define SETTLE_TIMEOUT_MS   (1000)   // Max time to wait for ADC to stabilize during measurement
#define SETTLE_TOLERANCE_ADC (5)     // ADC value must not change by more than this to be "stable"
#define LOOP_DELAY_MS       (10)     // Delay in measurement loops
#define FINAL_VALUE_TOLERANCE_ADC (20)  //Checks if the Tolerance it settled on is the correct ADC Value or not

// Sensor address (assuming it's defined elsewhere, or define it here)
#define SENSOR_ADDRESS      (0x48 << 1)

// Externs
extern I2C_HandleTypeDef hi2c1;
extern volatile uint16_t Pot1_2[2]; // Assuming this holds the ADC values

// ===================================================================
// 2. MODULE-LEVEL (STATIC) VARIABLES FOR CALIBRATION DATA
// ===================================================================
// These variables will store the results from the initial calibration run.
static uint16_t g_min_adc_value = 0;
static uint8_t  g_min_combination = 0;
static uint16_t g_max_adc_value = 0;
static uint8_t  g_max_combination = 0;
static bool     g_is_calibrated = false;


float min_voltage = FLT_MAX;
float max_voltage = 0.0f;



// ===================================================================
// 3. PRIVATE HELPER FUNCTIONS
// ===================================================================

/**
 * @brief Deactivates all three relays.
 */
void relay_reset(void) {
    HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_1PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_2PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_3PIN, GPIO_PIN_RESET);
}

/**
 * @brief Sets the state of a single, specific relay. (Your original function)
 */
void relay_set_state(Relay_ID_t relayID, Relay_State_t state) {
    GPIO_PinState pin_state = (state == RELAY_ON) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    switch (relayID) {
        case RELAY_1: HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_1PIN, pin_state); break;
        case RELAY_2: HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_2PIN, pin_state); break;
        case RELAY_3: HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_3PIN, pin_state); break;
        default: break;
    }
}

/**
 * @brief Sets relays to a specific combination based on a bitmask.
 * @param combination An 8-bit integer where bits 0, 1, and 2 correspond to relays 1, 2, and 3.
 */
static void set_relay_combination(uint8_t combination)
{
    // Use the bits of 'combination' to set each relay state
    relay_set_state(RELAY_1, (combination & 0b001) ? RELAY_ON : RELAY_OFF);
    relay_set_state(RELAY_2, (combination & 0b010) ? RELAY_ON : RELAY_OFF);
    relay_set_state(RELAY_3, (combination & 0b100) ? RELAY_ON : RELAY_OFF);
}


// ===================================================================
// 4. PUBLIC API FUNCTIONS (Declared in functions.h)
// ===================================================================

/**
 * @brief CALIBRATION: Cycles through all relay combinations to find the ones that produce
 *        the minimum and maximum ADC readings. This MUST be run before other tests.
 * @return HAL_StatusTypeDef HAL_OK on success.
 */
HAL_StatusTypeDef Calibrate_And_Find_Ranges()
{
    float current_voltage = 0.0f;

    printf("--- Starting Relay Calibration ---\r\n");
    relay_reset();
    HAL_Delay(SETTLE_DELAY_MS);

    for (int i = 0; i < 8; i++)
    {
        set_relay_combination(i);
        HAL_Delay(SETTLE_DELAY_MS); // Wait for the electronics to settle

        // Read the resulting ADC value (we use the raw ADC for consistency)
        uint16_t current_adc = Pot1_2[0]; // Assuming ADC is read into this global array

        // Also read voltage for logging
        I2C_ReadVoltage(&hi2c1, SENSOR_ADDRESS, &current_voltage);

        printf("State %d -> ADC: %u\r\n", i, current_adc);

        if (current_adc < g_min_adc_value || i == 0) {
            g_min_adc_value = current_adc;
            g_min_combination = i;
            min_voltage = current_voltage;
        }
        if (current_adc > g_max_adc_value || i == 0) {
            g_max_adc_value = current_adc;
            g_max_combination = i;
            max_voltage = current_voltage;
        }
    }

    g_is_calibrated = true;
    relay_reset();
    printf("\n--- Calibration Complete ---\r\n");
    printf("Min State: Combination %u -> ADC %u\r\n", g_min_combination, g_min_adc_value);
    printf("Max State: Combination %u -> ADC %u\r\n", g_max_combination, g_max_adc_value);
    printf("-----------------------------\r\n");

    return HAL_OK;
}

/**
 * @brief TEST 1: Measures the time it takes for the ADC value to settle after
 *        switching from the MIN state to the MAX state. It also verifies
 *        that the final settled value is close to the calibrated maximum.
 * @return Settling time in ms. Returns SETTLE_TIMEOUT_MS on timeout or if
 *         the final value is incorrect.
 */
uint32_t Measure_Settle_Time_Min_To_Max(void)
{
    if (!g_is_calibrated) {
        printf("ERROR: System not calibrated!\r\n");
        return 0;
    }
    relay_reset();

    // 1. Go to the starting state and wait
    set_relay_combination(g_min_combination);
    HAL_Delay(SETTLE_DELAY_MS);
    uint16_t last_adc = Pot1_2[0];

    // 2. Start timer and switch to the target state
    uint32_t start_tick = HAL_GetTick();
    set_relay_combination(g_max_combination);

    // 3. Loop until the ADC value is stable or we time out
    while (HAL_GetTick() - start_tick < SETTLE_TIMEOUT_MS)
    {
        uint16_t current_adc = Pot1_2[0];

        // Check if the reading has stabilized
        if (abs(current_adc - last_adc) <= SETTLE_TOLERANCE_ADC) {
            // --- STABILITY ACHIEVED ---
            uint32_t elapsed_time = HAL_GetTick() - start_tick;

            // <<< YOUR EXCELLENT SUGGESTION IMPLEMENTED HERE >>>
            // Now, verify if this stable value is correct.
            if (abs(current_adc - g_max_adc_value) <= FINAL_VALUE_TOLERANCE_ADC) {
                // SUCCESS: It's stable AND correct. Return the time.
            	relay_reset();
                return elapsed_time;
            } else {
                // FAILURE: It's stable, but at the WRONG value.
                printf("ERROR: Settled at incorrect ADC value! Expected ~%u, got %u\r\n",
                       g_max_adc_value, current_adc);
                return SETTLE_TIMEOUT_MS; // Return timeout to indicate failure
            }
        }

        last_adc = current_adc;
        HAL_Delay(LOOP_DELAY_MS);
    }

    // 4. If we reach here, the measurement timed out before becoming stable
    printf("ERROR: Timed out waiting for ADC to stabilize.\r\n");
    return SETTLE_TIMEOUT_MS;
}

/**
 * @brief TEST 2: Measures the time it takes for the ADC value to settle after
 *        switching from the MAX state to the MIN state. It also verifies
 *        that the final settled value is close to the calibrated minimum.
 * @return Settling time in ms. Returns SETTLE_TIMEOUT_MS on timeout or if
 *         the final value is incorrect.
 */
uint32_t Measure_Settle_Time_Max_To_Min(void)
{
    if (!g_is_calibrated) {
        printf("ERROR: System not calibrated!\r\n");
        return 0;
    }

    // 1. Go to the starting state and wait
    set_relay_combination(g_max_combination);
    HAL_Delay(SETTLE_DELAY_MS);
    uint16_t last_adc = Pot1_2[0];

    // 2. Start timer and switch to the target state
    uint32_t start_tick = HAL_GetTick();
    set_relay_combination(g_min_combination);

    // 3. Loop until the ADC value is stable or we time out
    while (HAL_GetTick() - start_tick < SETTLE_TIMEOUT_MS)
    {
        uint16_t current_adc = Pot1_2[0];

        // Check if the reading has stabilized
        if (abs(current_adc - last_adc) <= SETTLE_TOLERANCE_ADC) {
            // --- STABILITY ACHIEVED ---
            uint32_t elapsed_time = HAL_GetTick() - start_tick;

            // Now, verify if this stable value is correct.
            if (abs(current_adc - g_min_adc_value) <= FINAL_VALUE_TOLERANCE_ADC) {
                // SUCCESS: It's stable AND correct. Return the time.
                return elapsed_time;
            } else {
                // FAILURE: It's stable, but at the WRONG value.
                printf("ERROR: Settled at incorrect ADC value! Expected ~%u, got %u\r\n",
                       g_min_adc_value, current_adc);
                return SETTLE_TIMEOUT_MS; // Return timeout to indicate failure
            }
        }

        last_adc = current_adc;
        HAL_Delay(LOOP_DELAY_MS);
    }

    // 4. If we reach here, the measurement timed out before becoming stable
    printf("ERROR: Timed out waiting for ADC to stabilize.\r\n");
    return SETTLE_TIMEOUT_MS;
}

/**
 * @brief TEST 3 (SMOOTHNESS ANALOG): Measures the repeatability of the MIN state.
 *        It goes to MIN, then MAX, then back to MIN and measures the difference
 *        in the ADC reading.
 * @return The absolute difference in ADC counts. A low number (e.g., < 5) is good.
 */
uint16_t Measure_State_Repeatability_Min(void)
{
    if (!g_is_calibrated) {
        printf("ERROR: System not calibrated!\r\n");
        return 0xFFFF; // Return max error
    }
    relay_reset();
    HAL_Delay(1000);

    // 1. Go to MIN state and record the ADC value
    set_relay_combination(g_min_combination);
    HAL_Delay(SETTLE_DELAY_MS);
    uint16_t adc_first_reading = Pot1_2[0];

    // 2. Move to a different state (MAX)
    set_relay_combination(g_max_combination);
    HAL_Delay(SETTLE_DELAY_MS);

    // 3. Return to MIN state and record the new ADC value
    set_relay_combination(g_min_combination);
    HAL_Delay(SETTLE_DELAY_MS);
    uint16_t adc_second_reading = Pot1_2[0];

    // 4. Return the absolute difference
    return abs(adc_first_reading - adc_second_reading);
}

/**
 * @brief (Private Helper) Switches relay state and measures the peak ADC deviation
 *        from the final target value during the transition.
 * @param start_combination The relay combination to start from.
 * @param end_combination   The relay combination to switch to.
 * @param end_target_adc    The expected final ADC value for the end_combination.
 * @param max_deviation_out Pointer to store the maximum signed deviation (overshoot/undershoot).
 */
static void Measure_Transition_Deviation(uint8_t start_combination,
                                         uint8_t end_combination,
                                         uint16_t end_target_adc,
                                         int32_t* max_deviation_out)
{
    if (!g_is_calibrated) {
        *max_deviation_out = 0;
        return;
    }

    *max_deviation_out = 0;

    // 1. Go to the starting state and allow everything to settle.
    set_relay_combination(start_combination);
    HAL_Delay(SETTLE_DELAY_MS);
    uint16_t last_adc = Pot1_2[0];

    // 2. Start timer and switch to the target state.
    uint32_t start_tick = HAL_GetTick();
    set_relay_combination(end_combination);

    // 3. Loop until the ADC value is stable or we time out.
    //    During this time, we look for the peak deviation.
    while (HAL_GetTick() - start_tick < SETTLE_TIMEOUT_MS)
    {
        uint16_t current_adc = Pot1_2[0];

        // Calculate the deviation from the FINAL target value.
        int32_t current_deviation = (int32_t)current_adc - (int32_t)end_target_adc;

        // Check if this is the largest deviation we've seen (in magnitude).
        if (labs(current_deviation) > labs(*max_deviation_out)) {
            *max_deviation_out = current_deviation;
        }

        // Use the stability check as our exit condition.
        if (abs(current_adc - last_adc) <= SETTLE_TOLERANCE_ADC) {
            // Optional: Also check if the final value is correct.
            if (abs(current_adc - end_target_adc) <= FINAL_VALUE_TOLERANCE_ADC) {
                return; // Settled and correct, exit loop.
            }
        }

        last_adc = current_adc;
        HAL_Delay(LOOP_DELAY_MS);
    }
}

/**
 * @brief TEST 4: Measures the normalized overshoot error when switching from the MIN to MAX state.
 *        This is the relay-system equivalent of "smoothness."
 * @return A normalized error value (0.0 to 1.0+). 0 means a perfect transition with no overshoot.
 *         Returns a negative value on error (e.g., not calibrated).
 */
float Get_Relay_Min_to_Max_Overshoot_Error(void)
{
    if (!g_is_calibrated) {
        printf("ERROR: System not calibrated!\r\n");
        return -1.0f;
    }

    int32_t max_deviation_adc;
    Measure_Transition_Deviation(g_min_combination, g_max_combination, g_max_adc_value, &max_deviation_adc);

    // For a Min->Max transition, we only care about overshoot (positive deviation).
    // A negative deviation is just part of the normal rise time.
    if (max_deviation_adc < 0) {
        max_deviation_adc = 0;
    }

    // Normalize the error against the total operational range.
    float total_range = (float)(g_max_adc_value - g_min_adc_value);
    if (total_range <= 0) {
        return 0.0f;
    }

    float normalized_error = (float)max_deviation_adc / total_range;
    return normalized_error;
}

/**
 * @brief TEST 5: Measures the normalized undershoot error when switching from the MAX to MIN state.
 *        This is the relay-system equivalent of "smoothness."
 * @return A normalized error value (0.0 to 1.0+). 0 means a perfect transition with no undershoot.
 *         Returns a negative value on error (e.g., not calibrated).
 */
float Get_Relay_Max_to_Min_Undershoot_Error(void)
{
    if (!g_is_calibrated) {
        printf("ERROR: System not calibrated!\r\n");
        return -1.0f;
    }

    int32_t max_deviation_adc;
    Measure_Transition_Deviation(g_max_combination, g_min_combination, g_min_adc_value, &max_deviation_adc);

    // For a Max->Min transition, we only care about undershoot (negative deviation).
    // A positive deviation is just part of the normal fall time.
    if (max_deviation_adc > 0) {
        max_deviation_adc = 0;
    }

    // Normalize the error against the total operational range. Use labs() for magnitude.
    float total_range = (float)(g_max_adc_value - g_min_adc_value);
    if (total_range <= 0) {
        return 0.0f;
    }

    float normalized_error = (float)labs(max_deviation_adc) / total_range;
    return normalized_error;
}

