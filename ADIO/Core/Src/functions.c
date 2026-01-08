/*
 * functions.c
 *
 *  Created on: Jun 1, 2025
 *      Author: Aditya Singh
 */
#include "stm32f4xx_hal.h"
#include "main.h"
#include "functions.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "i2ccomm.h"
#include <math.h>
#include "DAC_Generator.h"

// ===================================================================
// 1. CONFIGURATION CONSTANTS (Refactored from hardcoded values)
// ===================================================================
#define MOTOR1_IN1          GPIO_PIN_7
#define MOTOR1_IN2          GPIO_PIN_8
#define MOTOR2_IN1          GPIO_PIN_9
#define MOTOR2_IN2          GPIO_PIN_10


#define DIO_GPIO_PORT     GPIOC

#define MODE_SELECT             GPIO_PIN_2

#define TAP_SELECT_BIT2         GPIO_PIN_1
#define TAP_SELECT_BIT1         GPIO_PIN_0
#define TAP_SELECT_BIT0         GPIO_PIN_3



#define MIN_TOLERANCE    20

#define MOVE_TIMEOUT_MS     (5000)    //CHANGED: Timeout for any motor movement
#define SETTLE_DELAY_MS     (100)     //CHANGED: Delay for motor to settle
#define LOOP_DELAY_MS       (1)      //Delay in motor control loops

float POS_MIN_TUNE = 0;
float POS_MAX_TUNE = 10;
float POS_HOME_TUNE = 5;

float POS_MIN_LOAD = 0;
float POS_MAX_LOAD = 10;
float POS_HOME_LOAD = 5;



uint16_t ADC_POS_MIN_TUNE = 0;
uint16_t ADC_POS_MAX_TUNE = 10;
uint16_t ADC_POS_HOME_TUNE = 5;

uint16_t ADC_POS_MIN_LOAD = 0;
uint16_t ADC_POS_MAX_LOAD = 10;
uint16_t ADC_POS_HOME_LOAD = 5;

uint16_t tolerance = MIN_TOLERANCE;

extern I2C_HandleTypeDef hi2c1;
extern DAC_HandleTypeDef hdac;
extern float vref;
float vref = 10.0f;






void ModeControlADIOBoard(DIO_t STATE  ){
	HAL_GPIO_WritePin(DIO_GPIO_PORT ,MODE_SELECT,STATE );
}


void relay_reset(void) {
    HAL_GPIO_WritePin(DIO_GPIO_PORT, TAP_SELECT_BIT0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIO_GPIO_PORT, TAP_SELECT_BIT1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIO_GPIO_PORT, TAP_SELECT_BIT2, GPIO_PIN_RESET);
}

void relay_set_state(uint16_t GPIO_Pin, DIO_t state)
{
    GPIO_PinState pin_state = (state == ON) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(DIO_GPIO_PORT, GPIO_Pin, pin_state);
}

void set_relay_combination(uint8_t combination)
{
	relay_reset();
    // Use the bits of 'combination' to set each relay state
    relay_set_state(TAP_SELECT_BIT0, (combination & 0b001) ? ON : OFF);
    relay_set_state(TAP_SELECT_BIT1, (combination & 0b010) ? ON : OFF);
    relay_set_state(TAP_SELECT_BIT2, (combination & 0b100) ? ON : OFF);
}


void TuneCoil(float targetPos,uint16_t targetADC)
{
    uint16_t currentADC = 0;
    int32_t error = 0;
    uint32_t timeout_start = HAL_GetTick();
    float Inst_Current;
    float Abs_Inst_Current_P;
    float Abs_Inst_Current_N;
    Max_PCurrent_Tune = 0;
    Max_NCurrent_Tune = 0;
    while (1) {
        // --- Read current for monitoring ---
        I2C_ReadCurrent(&hi2c1, P15, &Inst_Current);
        Abs_Inst_Current_P = fabs(Inst_Current);
        if (Abs_Inst_Current_P > Max_PCurrent_Tune)
            Max_PCurrent_Tune = Abs_Inst_Current_P;
        I2C_ReadCurrent(&hi2c1, N15, &Inst_Current);
        Abs_Inst_Current_N = fabs(Inst_Current);
        if (Abs_Inst_Current_N > Max_NCurrent_Tune)
            Max_NCurrent_Tune = Abs_Inst_Current_N;
        // --- Read current position ---
        currentADC = ADC_IN[0];
        error = (int32_t)targetADC - (int32_t)currentADC;

        // --- Check if target reached ---
        if (labs(error) <= tolerance) {

        	DACGenerator_Stop(&hdac,DAC_CHANNEL_1);
            return;
        }
        DACGenerator_Start(&hdac, DAC_CHANNEL_1, targetPos, vref );



        // --- Timeout protection --
        if (HAL_GetTick() - timeout_start > MOVE_TIMEOUT_MS) {

        	DACGenerator_Stop(&hdac,DAC_CHANNEL_1);
            return;
        }
        HAL_Delay(LOOP_DELAY_MS);
    }
}

void LoadCoil(float targetPos, uint16_t targetADC)
{
    uint16_t currentADC = 0;
    int32_t error = 0;
    uint32_t timeout_start = HAL_GetTick();
    float Inst_Current;
    float Abs_Inst_Current_P;
    float Abs_Inst_Current_N;
    Max_PCurrent_Load = 0;
    Max_NCurrent_Load = 0;


    while (1) {
        // --- Read current for monitoring ---
        I2C_ReadCurrent(&hi2c1, P15, &Inst_Current);
        Abs_Inst_Current_P = fabs(Inst_Current);
        if (Abs_Inst_Current_P > Max_PCurrent_Load)
            Max_PCurrent_Load = Abs_Inst_Current_P;
        I2C_ReadCurrent(&hi2c1, N15, &Inst_Current);
        Abs_Inst_Current_N = fabs(Inst_Current);
        if (Abs_Inst_Current_N > Max_NCurrent_Load)
            Max_NCurrent_Load = Abs_Inst_Current_N;
        // --- Read current position ---
        currentADC = ADC_IN[0];
        error = (int32_t)targetADC - (int32_t)currentADC;

        // --- Check if target reached ---
        if (labs(error) <= tolerance) {

        	DACGenerator_Stop(&hdac,DAC_CHANNEL_2);
            return;
        }
        DACGenerator_Start(&hdac, DAC_CHANNEL_2, targetPos, vref );

        // --- Timeout protection --
        if (HAL_GetTick() - timeout_start > MOVE_TIMEOUT_MS) {

        	DACGenerator_Stop(&hdac,DAC_CHANNEL_2);
            return;
        }
        HAL_Delay(LOOP_DELAY_MS);
    }
}


void TuneCoilSmoothness(float voltage,
                        uint16_t targetADC,
                        uint32_t expected_duration_ms,
                        uint16_t tolerance,
                        int32_t* max_error_out)
{
    const int32_t start_adc = ADC_IN[0];
    const int32_t end_adc   = targetADC;

    int32_t error;
    *max_error_out = 0;

    const uint32_t move_start_tick = HAL_GetTick();

    uint16_t current_adc;

    while (1)
    {
        // -------- SENSE --------
        current_adc = ADC_IN[0];
        error = end_adc - current_adc;

        // -------- CHECK TARGET REACHED --------
        if (labs(error) <= tolerance)
        {
            DACGenerator_Stop(&hdac, DAC_CHANNEL_1);
            return;
        }

        // -------- MEASURE SMOOTHNESS --------
        uint32_t elapsed_time = HAL_GetTick() - move_start_tick;
        if (elapsed_time > expected_duration_ms)
            elapsed_time = expected_duration_ms;

        if (expected_duration_ms > 0)
        {
            int32_t travel_distance = end_adc - start_adc;
            int32_t ideal_position =
                start_adc + (int32_t)((travel_distance * (int64_t)elapsed_time) /
                                       expected_duration_ms);

            int32_t smoothness_error = current_adc - ideal_position;

            if (labs(smoothness_error) > labs(*max_error_out))
                *max_error_out = smoothness_error;
        }

        // -------- ACT --------
        DACGenerator_Start(&hdac, DAC_CHANNEL_1, voltage, vref);

        // -------- TIMEOUT --------
        if (HAL_GetTick() - move_start_tick > MOVE_TIMEOUT_MS)
        {
            DACGenerator_Stop(&hdac, DAC_CHANNEL_1);
            return;
        }

        HAL_Delay(LOOP_DELAY_MS);
    }
}


void LoadCoilSmoothness(float voltage,
                        uint16_t targetADC,
                        uint32_t expected_duration_ms,
                        uint16_t tolerance,
                        int32_t* max_error_out)
{
    const int32_t start_adc = ADC_IN[0];
    const int32_t end_adc   = targetADC;

    int32_t error;
    *max_error_out = 0;

    const uint32_t move_start_tick = HAL_GetTick();

    uint16_t current_adc;

    while (1)
    {
        // -------- SENSE --------
        current_adc = ADC_IN[0];
        error = end_adc - current_adc;

        // -------- CHECK TARGET REACHED --------
        if (labs(error) <= tolerance)
        {
            DACGenerator_Stop(&hdac, DAC_CHANNEL_2);
            return;
        }

        // -------- MEASURE SMOOTHNESS --------
        uint32_t elapsed_time = HAL_GetTick() - move_start_tick;
        if (elapsed_time > expected_duration_ms)
            elapsed_time = expected_duration_ms;

        if (expected_duration_ms > 0)
        {
            int32_t travel_distance = end_adc - start_adc;
            int32_t ideal_position =
                start_adc + (int32_t)((travel_distance * (int64_t)elapsed_time) /
                                       expected_duration_ms);

            int32_t smoothness_error = current_adc - ideal_position;

            if (labs(smoothness_error) > labs(*max_error_out))
                *max_error_out = smoothness_error;
        }

        // -------- ACT --------
        DACGenerator_Start(&hdac, DAC_CHANNEL_2, voltage, vref);


        // -------- TIMEOUT --------
        if (HAL_GetTick() - move_start_tick > MOVE_TIMEOUT_MS)
        {
            DACGenerator_Stop(&hdac, DAC_CHANNEL_2);
            return;
        }

        HAL_Delay(LOOP_DELAY_MS);
    }
}



// ===================================================================
// 3. PUBLIC API FUNCTIONS (Declared in functions.h)
// ===================================================================
uint32_t ADC_MIN_TO_ADC_MAX_TUNE()
{
    // Move to min first
    TuneCoil(POS_MIN_TUNE, ADC_POS_MIN_TUNE);

    uint32_t timer_Start = HAL_GetTick();

    // Move to max
    TuneCoil(POS_MAX_TUNE, ADC_POS_MAX_TUNE);

    uint32_t timer_End = HAL_GetTick();

    parameters[PARAM_X1_P15V_I_MIN_MAX] = Max_PCurrent_Tune;
    parameters[PARAM_X1_N15V_I_MIN_MAX] = Max_NCurrent_Tune;

    uint32_t time_taken = timer_End - timer_Start;

    // Return to min
    TuneCoil(POS_MIN_TUNE, ADC_POS_MIN_TUNE);

    return time_taken;
}


uint32_t ADC_MAX_TO_ADC_MIN_TUNE()
{
    // Move to max first
    TuneCoil(POS_MAX_TUNE, ADC_POS_MAX_TUNE);

    uint32_t timer_Start = HAL_GetTick();

    // Move to min
    TuneCoil(POS_MIN_TUNE, ADC_POS_MIN_TUNE);

    uint32_t timer_End = HAL_GetTick();

    parameters[PARAM_X1_P15V_I_MIN_MAX] = Max_PCurrent_Tune;
    parameters[PARAM_X1_N15V_I_MIN_MAX] = Max_NCurrent_Tune;

    uint32_t time_taken = timer_End - timer_Start;

    // Go home
    TuneCoil(POS_HOME_TUNE, ADC_POS_HOME_TUNE);

    return time_taken;
}

float Get_Tune_Min_to_Max_Smoothness()
{
    uint32_t duration_ms = ADC_MIN_TO_ADC_MAX_TUNE();

    // Ensure starting from min
    TuneCoil(POS_MIN_TUNE, ADC_POS_MIN_TUNE);
    HAL_Delay(SETTLE_DELAY_MS);

    int32_t max_error_in_adc = 0;

    TuneCoilSmoothness(
        POS_MAX_TUNE,
        ADC_POS_MAX_TUNE,
        duration_ms,
        MOVE_TOLERANCE_ADC,
        &max_error_in_adc
    );

    float total_distance = (float)(ADC_POS_MAX_TUNE - ADC_POS_MIN_TUNE);
    if (total_distance == 0) return 0.0f;

    float normalized_error = (float)max_error_in_adc / total_distance;

    // Return to min
    TuneCoil(POS_MIN_TUNE, ADC_POS_MIN_TUNE);

    return normalized_error;
}

float Get_Tune_Max_to_Min_Smoothness(void)
{
    uint32_t duration_ms = ADC_MAX_TO_ADC_MIN_TUNE();

    // Ensure starting from max
    TuneCoil(POS_MAX_TUNE, ADC_POS_MAX_TUNE);
    HAL_Delay(SETTLE_DELAY_MS);

    int32_t max_error_in_adc = 0;

    TuneCoilSmoothness(
        POS_MIN_TUNE,
        ADC_POS_MIN_TUNE,
        duration_ms,
        MOVE_TOLERANCE_ADC,
        &max_error_in_adc
    );

    float total_distance = (float)(ADC_POS_MAX_TUNE - ADC_POS_MIN_TUNE);
    if (total_distance == 0) return 0.0f;

    float normalized_error = (float)max_error_in_adc / total_distance;

    // Return to min
    TuneCoil(POS_MIN_TUNE, ADC_POS_MIN_TUNE);

    return normalized_error;
}

uint32_t ADC_MIN_TO_ADC_MAX_LOAD()
{
    LoadCoil(POS_MIN_LOAD, ADC_POS_MIN_LOAD);

    uint32_t Motor2_Start = HAL_GetTick();

    LoadCoil(POS_MAX_LOAD, ADC_POS_MAX_LOAD);

    uint32_t Motor2_End = HAL_GetTick();

    parameters[PARAM_X2_P15V_I_MIN_MAX] = Max_PCurrent_Load;

    uint32_t time_taken = Motor2_End - Motor2_Start;

    LoadCoil(POS_MIN_LOAD, ADC_POS_MIN_LOAD);

    return time_taken;
}


uint32_t ADC_MAX_TO_ADC_MIN_M2()
{
    LoadCoil(POS_MAX_LOAD, ADC_POS_MAX_LOAD);

    uint32_t Motor2_Start = HAL_GetTick();

    LoadCoil(POS_MIN_LOAD, ADC_POS_MIN_LOAD);

    uint32_t Motor2_End = HAL_GetTick();

    parameters[PARAM_X2_P15V_I_MAX_MIN] = Max_PCurrent_Load;

    uint32_t time_taken = Motor2_End - Motor2_Start;

    LoadCoil(POS_MIN_LOAD, ADC_POS_MIN_LOAD);

    return time_taken;
}

float Get_M2_Min_to_Max_Smoothness()
{
    uint32_t duration_ms = ADC_MIN_TO_ADC_MAX_LOAD();

    LoadCoil(POS_MIN_LOAD, ADC_POS_MIN_LOAD);
    HAL_Delay(SETTLE_DELAY_MS);

    int32_t max_error_in_adc = 0;

    LoadCoilSmoothness(
        POS_MAX_LOAD,
        ADC_POS_MAX_LOAD,
        duration_ms,
        MOVE_TOLERANCE_ADC,
        &max_error_in_adc
    );

    float total_distance = (float)(ADC_POS_MAX_LOAD - ADC_POS_MIN_LOAD);
    if (total_distance == 0) return 0.0f;

    float normalized_error = (float)max_error_in_adc / total_distance;

    LoadCoil(POS_MIN_LOAD, ADC_POS_MIN_LOAD);

    return normalized_error;
}
float Get_M2_Max_to_Min_Smoothness(void)
{
    uint32_t duration_ms = ADC_MAX_TO_ADC_MIN_M2();

    LoadCoil(POS_MAX_LOAD, ADC_POS_MAX_LOAD);
    HAL_Delay(SETTLE_DELAY_MS);

    int32_t max_error_in_adc = 0;

    LoadCoilSmoothness(
        POS_MIN_LOAD,
        ADC_POS_MIN_LOAD,
        duration_ms,
        MOVE_TOLERANCE_ADC,
        &max_error_in_adc
    );

    float total_distance = (float)(ADC_POS_MAX_LOAD - ADC_POS_MIN_LOAD);
    if (total_distance == 0) return 0.0f;

    float normalized_error = (float)max_error_in_adc / total_distance;

    LoadCoil(POS_MIN_LOAD, ADC_POS_MIN_LOAD);

    return normalized_error;
}
