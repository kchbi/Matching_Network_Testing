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


#define DIO_GPIO_PORT     GPIOA



#define TAP_SELECT_BIT2         GPIO_PIN_9
#define MODE_SELECT             GPIO_PIN_10
#define TAP_SELECT_BIT1         GPIO_PIN_9
#define TAP_SELECT_BIT0         GPIO_PIN_9





#define MOVE_TIMEOUT_MS     (5000)    //CHANGED: Timeout for any motor movement
#define SETTLE_DELAY_MS     (100)     //CHANGED: Delay for motor to settle
#define LOOP_DELAY_MS       (1)      //Delay in motor control loops

uint16_t POS_MIN_TUNE = 530;
uint16_t POS_MAX_TUNE = 2230;
uint16_t POS_HOME_TUNE = 0;

uint16_t POS_MIN_LOAD = 530;
uint16_t POS_MAX_LOAD = 2230;
uint16_t POS_HOME_LOAD = 0;

extern I2C_HandleTypeDef hi2c1;




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
    // Use the bits of 'combination' to set each relay state
    relay_set_state(TAP_SELECT_BIT0, (combination & 0b001) ? ON : OFF);
    relay_set_state(TAP_SELECT_BIT1, (combination & 0b010) ? ON : OFF);
    relay_set_state(TAP_SELECT_BIT2, (combination & 0b100) ? ON : OFF);
}


void TuneCoil(float targetPos,uint16_t targetADC)
{
    uint16_t currentADC = 0;
    int32_t delta = 0;
    uint32_t timeout_start = HAL_GetTick();
    float Inst_Current;
    Max_PCurrent_Tune = 0;
    Max_NCurrent_Tune = 0;
    while (1) {
        // --- Read current for monitoring ---
        I2C_ReadCurrent(&hi2c1, P15, &Inst_Current);
        float Abs_Inst_Current = fabs(Inst_Current);
        if (Abs_Inst_Current > Max_PCurrent_Tune)
            Max_PCurrent_Tune = Abs_Inst_Current;
        I2C_ReadCurrent(&hi2c1, N15, &Inst_Current);
        float Abs_Inst_Current = fabs(Inst_Current);
        if (Abs_Inst_Current > Max_NCurrent_Tune)
            Max_NCurrent_Tune = Abs_Inst_Current;
        // --- Read current position ---
        currentADC = ADC_IN[0];
        delta = (int32_t)PreviousADC - (int32_t)currentADC;

        // --- Check if target reached ---
        if (labs(delta) <= tolerance) {

        	DACGenerator_Stop(&hdac,DAC_CHANNEL_1);
            return;
        }
        DACGenerator_Start(&hdac, DAC_CHANNEL_1, voltage, vref );
        PreviousADC = ADC_IN[0];


        // --- Timeout protection --
        if (HAL_GetTick() - timeout_start > MOVE_TIMEOUT_MS) {

        	DACGenerator_Stop(&hdac,DAC_CHANNEL_1);
            return;
        }
        HAL_Delay(LOOP_DELAY_MS);
    }
}


void LoadCoil(float targetPos)
{
    uint16_t currentADC = 0;
    int32_t delta = 0;
    uint32_t timeout_start = HAL_GetTick();
    float Inst_Current_P;
    float Inst_Current_N;
    Max_PCurrent_Coil = 0;
    Max_NCurrent_Coil = 0;


    while (1) {
        // --- Read current for monitoring ---
        I2C_ReadCurrent(&hi2c1, P15, &Inst_Current);
        float Abs_Inst_Current = fabs(Inst_Current);
        if (Abs_Inst_Current > Max_PCurrent_Coil)
            Max_PCurrent_Coil = Abs_Inst_Current;
        I2C_ReadCurrent(&hi2c1, N15, &Inst_Current);
        float Abs_Inst_Current = fabs(Inst_Current);
        if (Abs_Inst_Current > Max_NCurrent_Coil)
            Max_NCurrent_Coil = Abs_Inst_Current;
        // --- Read current position ---
        currentADC = ADC_IN[0];
        delta = (int32_t)targetADC - (int32_t)currentADC;

        // --- Check if target reached ---
        if (labs(delta) <= tolerance) {

        	DACGenerator_Stop(&hdac,DAC_CHANNEL_2);
            return;
        }
        DACGenerator_Start(&hdac, DAC_CHANNEL_2, voltage, vref );

        // --- Timeout protection --
        if (HAL_GetTick() - timeout_start > MOVE_TIMEOUT_MS) {

        	DACGenerator_Stop(&hdac,DAC_CHANNEL_2);
            return;
        }
        HAL_Delay(LOOP_DELAY_MS);
    }
}


void TuneCoilSmoothness(float voltage, uint32_t expected_duration_ms, uint16_t tolerance, int32_t* max_error_out)
{
    const int32_t start_adc = ADC_IN[0];

    int32_t delta;
    *max_error_out = 0;

    const uint32_t move_start_tick = HAL_GetTick();

    uint16_t current_adc = 0;
    delta = 0;

    while (1) {
        // --- SENSE ---
        current_adc = ADC_IN[0];
        delta = previous_ADC - current_adc;

        // --- CHECK FOR COMPLETION ---
        if (labs(delta) <= tolerance) {
        	DACGenerator_Stop(&hdac,DAC_CHANNEL_1);
            return;
        }

        // --- MEASURE SMOOTHNESS ---
        uint32_t elapsed_time = HAL_GetTick() - move_start_tick;
        if (expected_duration_ms > 0) {
            int32_t travel_distance = end_adc - start_adc;
            int32_t ideal_position = start_adc + (int32_t)(((long long)travel_distance * elapsed_time) / expected_duration_ms);
            int32_t current_smoothness_error = current_adc - ideal_position;
            if (labs(current_smoothness_error) > labs(*max_error_out)) {
                *max_error_out = current_smoothness_error;
            }
        }
        // --- ACT ---
        DACGenerator_Start(&hdac, DAC_CHANNEL_1, voltage, vref );
        previous_ADC=ADC_IN[0];

        // --- TIMEOUT PROTECTION ---
        if (HAL_GetTick() - move_start_tick > MOVE_TIMEOUT_MS) {
        	DACGenerator_Stop(&hdac,DAC_CHANNEL_1);
            return;
        }
        HAL_Delay(LOOP_DELAY_MS);
    }
}
void LoadCoilSmoothness(float voltage, uint32_t expected_duration_ms, uint16_t tolerance, int32_t* max_error_out)
{
    const int32_t start_adc = ADC_IN[0];

    int32_t delta;
    *max_error_out = 0;

    const uint32_t move_start_tick = HAL_GetTick();


    uint16_t current_adc = 0; // Initialize to satisfy compiler warnings
    delta = 0;      // Initialize to satisfy compiler warnings

    while (1) {
        // --- SENSE ---
        current_adc = ADC_IN[0];
        delta = previous_ADC - current_adc;

        // --- CHECK FOR COMPLETION ---
        if (labs(delta) <= tolerance) {
        	DACGenerator_Stop(&hdac,DAC_CHANNEL_2);
            return;
        }

        // --- MEASURE SMOOTHNESS ---
        uint32_t elapsed_time = HAL_GetTick() - move_start_tick;
        if (expected_duration_ms > 0) {
            int32_t travel_distance = end_adc - start_adc;
            int32_t ideal_position = start_adc + (int32_t)(((long long)travel_distance * elapsed_time) / expected_duration_ms);
            int32_t current_smoothness_error = current_adc - ideal_position;
            if (labs(current_smoothness_error) > labs(*max_error_out)) {
                *max_error_out = current_smoothness_error;
            }
        }
        // --- ACT ---
        DACGenerator_Start(&hdac, DAC_CHANNEL_2, voltage, vref );
        previous_ADC=ADC_IN[0];

        // --- TIMEOUT PROTECTION ---
        if (HAL_GetTick() - move_start_tick > MOVE_TIMEOUT_MS) {
        	DACGenerator_Stop(&hdac,DAC_CHANNEL_2);
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
    // float voltage_reading; // <<< REMOVED

    TuneCoil(POS_MIN_TUNE);
    uint32_t timer_Start = HAL_GetTick();
    TuneCoil(POS_MAX_M1);
    uint32_t timer_End = HAL_GetTick();
    parameters[PARAM_X1_P15V_I_MIN_MAX] = Max_PCurrent_Tune;
    parameters[PARAM_X1_N15V_I_MIN_MAX] = Max_NCurrent_Tune;

    uint32_t time_taken = timer_End - timer_Start;
    TuneCoil(ADC_POS_MIN_M1);
    return time_taken;
}

uint32_t ADC_MAX_TO_ADC_MIN_TUNE()
{
    TuneCoil(POS_MAX_TUNE);
    uint32_t Timer_Start = HAL_GetTick();
    TuneCoil(POS_MIN_TUNE);
    uint32_t Timer_End = HAL_GetTick();
    parameters[PARAM_X1_P15V_I_MIN_MAX] = Max_PCurrent_Tune;
    parameters[PARAM_X1_N15V_I_MIN_MAX] = Max_NCurrent_Tune;
    uint32_t time_taken = Motor1_End - Motor1_Start;
    TuneCoil(POS_HOME_TUNE);
    return time_taken;
}

float Get_Tune_Min_to_Max_Smoothness()
{
    uint32_t duration_ms = ADC_MIN_TO_ADC_MAX_TUNE();
    TuneCoil(POS_MIN_TUNE);
    HAL_Delay(SETTLE_DELAY_MS);
    int32_t max_error_in_adc;
    TuneCoilSmoothness(POS_MAX_TUNE, duration_ms, MOVE_TOLERANCE_ADC, &max_error_in_adc);
    float total_distance = (float)(ADC_POS_MAX_M1 - ADC_POS_MIN_M1);
    if (total_distance == 0) return 0.0f;
    float normalized_error = (float)max_error_in_adc / total_distance;
    TuneCoil(POS_MIN_M1);
    return normalized_error;
}

float Get_Tune_Max_to_Min_Smoothness(void)
{
    uint32_t duration_ms = ADC_MAX_TO_ADC_MIN_TUNE();
    moveMotor1ToADCValue(ADC_POS_MAX_M1, MOVE_TOLERANCE_ADC);
    HAL_Delay(SETTLE_DELAY_MS);
    int32_t max_error_in_adc;
    moveMotor1ToADCValue_And_Measure_Smoothness(ADC_POS_MIN_M1, duration_ms, MOVE_TOLERANCE_ADC, &max_error_in_adc);
    float total_distance = (float)(ADC_POS_MAX_M1 - ADC_POS_MIN_M1);
    if (total_distance == 0) return 0.0f;
    float normalized_error = (float)max_error_in_adc / total_distance;
    moveMotor1ToADCValue(ADC_POS_MIN_M1, MOVE_TOLERANCE_ADC);
    return normalized_error;
}

uint32_t ADC_MIN_TO_ADC_MAX_LOAD()
{
    moveMotor2ToADCValue(ADC_POS_MIN_M2);
    uint32_t Motor2_Start = HAL_GetTick();
    moveMotor2ToADCValue(ADC_POS_MAX_M2, MOVE_TOLERANCE_ADC);
    uint32_t Motor2_End = HAL_GetTick();

    parameters[PARAM_X2_P15V_I_MIN_MAX] = Max_PCurrent_M2;

    uint32_t time_taken = Motor2_End - Motor2_Start;
    moveMotor2ToADCValue(ADC_POS_MIN_M2, MOVE_TOLERANCE_ADC);
    return time_taken;
}

uint32_t ADC_MAX_TO_ADC_MIN_M2()
{
    // float voltage_reading; // <<< REMOVED

    moveMotor2ToADCValue(ADC_POS_MAX_M2, MOVE_TOLERANCE_ADC);
    uint32_t Motor2_Start = HAL_GetTick();
    moveMotor2ToADCValue(ADC_POS_MIN_M2, MOVE_TOLERANCE_ADC);
    uint32_t Motor2_End = HAL_GetTick();

    // <<< REMOVED: This entire block was using the deleted variable
    // if (I2C_ReadVoltage(&hi2c1, Pot2, &voltage_reading) == HAL_OK)
    // {
    //     parameters[PARAM_X2_MIN_POS_V] = voltage_reading;
    // }

    parameters[PARAM_X2_P15V_I_MAX_MIN] = Max_PCurrent_M2;

    uint32_t time_taken = Motor2_End - Motor2_Start;
    moveMotor2ToADCValue(ADC_POS_MIN_M2, MOVE_TOLERANCE_ADC);
    return time_taken;
}

float Get_M2_Min_to_Max_Smoothness()
{
    uint32_t duration_ms = ADC_MIN_TO_ADC_MAX_M2();
    moveMotor2ToADCValue(ADC_POS_MIN_M2, MOVE_TOLERANCE_ADC);
    HAL_Delay(SETTLE_DELAY_MS);
    int32_t max_error_in_adc;
    moveMotor2ToADCValue_And_Measure_Smoothness(ADC_POS_MAX_M2, duration_ms, MOVE_TOLERANCE_ADC, &max_error_in_adc);
    float total_distance = (float)(ADC_POS_MAX_M2 - ADC_POS_MIN_M2);
    if (total_distance == 0) return 0.0f;
    float normalized_error = (float)max_error_in_adc / total_distance;
    moveMotor2ToADCValue(ADC_POS_MIN_M2, MOVE_TOLERANCE_ADC);
    return normalized_error;
}

float Get_M2_Max_to_Min_Smoothness(void)
{
    uint32_t duration_ms = ADC_MAX_TO_ADC_MIN_M2();
    moveMotor2ToADCValue(ADC_POS_MAX_M2, MOVE_TOLERANCE_ADC);
    HAL_Delay(SETTLE_DELAY_MS);
    int32_t max_error_in_adc;
    moveMotor2ToADCValue_And_Measure_Smoothness(ADC_POS_MIN_M2, duration_ms, MOVE_TOLERANCE_ADC, &max_error_in_adc);
    float total_distance = (float)(ADC_POS_MAX_M2 - ADC_POS_MIN_M2);
    if (total_distance == 0) return 0.0f;
    float normalized_error = (float)max_error_in_adc / total_distance;
    moveMotor2ToADCValue(ADC_POS_MIN_M2, MOVE_TOLERANCE_ADC);
    return normalized_error;
}
