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

// ===================================================================
// 1. CONFIGURATION CONSTANTS (Refactored from hardcoded values)
// ===================================================================
#define MOTOR1_IN1          GPIO_PIN_7
#define MOTOR1_IN2          GPIO_PIN_8
#define MOTOR2_IN1          GPIO_PIN_9
#define MOTOR2_IN2          GPIO_PIN_11

#define MOTOR_SPEED         (200)     //CHANGED: Centralized speed setting
#define PWM_MAX_DUTY        (200)     //CHANGED: Matched to timer period for full range
#define MOVE_TIMEOUT_MS     (5000)    //CHANGED: Timeout for any motor movement
#define SETTLE_DELAY_MS     (100)     //CHANGED: Delay for motor to settle
#define LOOP_DELAY_MS       (10)      //Delay in motor control loops


extern I2C_HandleTypeDef hi2c1;
// ===================================================================
// 2. PRIVATE HELPER FUNCTIONS (Marked as 'static')
// ===================================================================
// These functions are only used inside this file.

void motor1_set_state(MotorDirection_t direction, uint32_t speed)
{
    if (speed > PWM_MAX_DUTY) {
        speed = PWM_MAX_DUTY;
    }
    switch (direction) {
        case MOTOR_FORWARD:
            HAL_GPIO_WritePin(GPIOE, MOTOR1_IN1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOE, MOTOR1_IN2, GPIO_PIN_RESET);
            TIM1->CCR1 = speed;
            break;
        case MOTOR_REVERSE:
            HAL_GPIO_WritePin(GPIOE, MOTOR1_IN1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOE, MOTOR1_IN2, GPIO_PIN_SET);
            TIM1->CCR1 = speed;
            break;
        case MOTOR_STOP:
            HAL_GPIO_WritePin(GPIOE, MOTOR1_IN1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOE, MOTOR1_IN2, GPIO_PIN_SET);
            TIM1->CCR1 = speed;
            break;
    }
}

void motor2_set_state(MotorDirection_t direction, uint32_t speed)
{
    if (speed > PWM_MAX_DUTY) {
        speed = PWM_MAX_DUTY;
    }
    switch (direction) {
        case MOTOR_FORWARD:
            HAL_GPIO_WritePin(GPIOE, MOTOR2_IN1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOE, MOTOR2_IN2, GPIO_PIN_RESET);
            TIM2->CCR1 = speed;
            break;
        case MOTOR_REVERSE:
            HAL_GPIO_WritePin(GPIOE, MOTOR2_IN1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOE, MOTOR2_IN2, GPIO_PIN_SET);
            TIM2->CCR1 = speed;
            break;
        case MOTOR_STOP:
            HAL_GPIO_WritePin(GPIOE, MOTOR2_IN1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOE, MOTOR2_IN2, GPIO_PIN_SET);
            TIM2->CCR1 = speed;
            break;
    }
}

/**
 * @brief Moves Motor 1 to a specified ADC target position.
 * @param targetADC The target ADC value (position) to reach.
 * @param tolerance The acceptable error range around the target.
 */
void moveMotor1ToADCValue(uint16_t targetADC, uint16_t tolerance)
{
    uint16_t currentADC;
    int32_t error;
    uint32_t timeout_start = HAL_GetTick();
    float Inst_CurrentP;
    float Inst_CurrentN;

    // Reset the global variables for Motor 1 before starting the move
    Max_PCurrent_M1 = 0;
    Max_NCurrent_M1 = 0;
    Voltage_M1 = 0; // Assuming you have this variable as well

    while (1) {
        // --- FOR MOTOR 1: Read the potentiometer for Motor 1 ---
        currentADC = Pot1_2[0];
        error = (int32_t)targetADC - (int32_t)currentADC;

        // --- FOR MOTOR 1: Read positive current and update max value ---
        I2C_ReadCurrent(&hi2c1, Motor1P, &Inst_CurrentP);

        // --- CORRECTED: Check the local variable 'Inst_CurrentP', not 'Inst_Current' ---
        if (Inst_CurrentP > 0) {
            if (Inst_CurrentP > Max_PCurrent_M1)
            {
                Max_PCurrent_M1 = Inst_CurrentP;
            }
        }

        // --- FOR MOTOR 1: Read negative current and update max value ---
        I2C_ReadCurrent(&hi2c1, Motor1N, &Inst_CurrentN);

        // --- CORRECTED: Check 'Inst_CurrentN' and update 'Max_NCurrent_M1' ---
        if (Inst_CurrentN > Max_NCurrent_M1)
        {
            Max_NCurrent_M1 = Inst_CurrentN; // This now updates the correct variable
        }

        // Check if the motor is at the target position
        if (labs(error) <= tolerance) {
            motor1_set_state(MOTOR_STOP, MOTOR_SPEED);
            return; // Exit the function, move complete
        }

        // Move the motor in the correct direction
        if (error > 0) {
            // Target is greater than current, motor needs to move to increase ADC
            motor1_set_state(MOTOR_FORWARD, MOTOR_SPEED);
        } else {
            // Target is less than current, motor needs to move to decrease ADC
            motor1_set_state(MOTOR_REVERSE, MOTOR_SPEED);
        }

        // Check for timeout to prevent the motor from running forever
        if (HAL_GetTick() - timeout_start > MOVE_TIMEOUT_MS) {
            motor1_set_state(MOTOR_STOP, MOTOR_SPEED);
            // You might want to set an error flag here to indicate a timeout occurred
            return; // Exit the function
        }

        HAL_Delay(LOOP_DELAY_MS);
    }
}

/**
 * @brief Moves Motor 2 to a specified ADC target position.
 * @param targetADC The target ADC value (position) to reach.
 * @param tolerance The acceptable error range around the target.
 */
void moveMotor2ToADCValue(uint16_t targetADC, uint16_t tolerance)
{
    uint16_t currentADC;
    int32_t error;
    uint32_t timeout_start = HAL_GetTick();
    float Inst_CurrentP;
    float Inst_CurrentN;
    //float Inst_CurrentPo;

    // Reset the correct global variables for Motor 2
    Max_PCurrent_M2 = 0;
    Max_NCurrent_M2 = 0;
    Max_PoCurrent_M2 = 0;
    Voltage_M2 = 0; // Assuming you have a voltage variable for motor 2

    while (1) {
        // --- CHANGE: Read the potentiometer for Motor 2 ---
        currentADC = Pot1_2[1];
        error = (int32_t)targetADC - (int32_t)currentADC;

        // --- CHANGE: Use I2C addresses and global variables for Motor 2 ---
        I2C_ReadCurrent(&hi2c1, Motor2P, &Inst_CurrentP);

        // --- BUG FIX 1: Correctly check the local variable Inst_CurrentP ---
        if (Inst_CurrentP > 0) {
            if (Inst_CurrentP > Max_PCurrent_M2)
            {
                Max_PCurrent_M2 = Inst_CurrentP;
            }
        }

        // --- CHANGE: Use I2C addresses for Motor 2 ---
        I2C_ReadCurrent(&hi2c1, Motor2N, &Inst_CurrentN);

        // --- BUG FIX 2 & CHANGE: Correctly check Inst_CurrentN and update Max_NCurrent_M2 ---
        if (Inst_CurrentN > Max_NCurrent_M2)
        {
            Max_NCurrent_M2 = Inst_CurrentN;
        }
//        I2C_ReadCurrent(&hi2c1, Pot2, &Inst_CurrentPo);
//
//        // --- BUG FIX 2 & CHANGE: Correctly check Inst_CurrentN and update Max_NCurrent_M2 ---
//        if (Inst_CurrentPo > Max_PoCurrent_M2)
//        {
//            Max_PoCurrent_M2 = Inst_CurrentPo;
//        }

        // Check if the motor is at the target position
        if (labs(error) <= tolerance) {
            motor2_set_state(MOTOR_STOP, MOTOR_SPEED); // CHANGE
            return; // Exit the function
        }

        // Move the motor in the correct direction
        if (error > 0) {
            // Target is greater than current, motor needs to move to increase ADC
            motor2_set_state(MOTOR_FORWARD, MOTOR_SPEED); // CHANGE
        } else {
            // Target is less than current, motor needs to move to decrease ADC
            motor2_set_state(MOTOR_REVERSE, MOTOR_SPEED); // CHANGE
        }

        // Check for timeout
        if (HAL_GetTick() - timeout_start > MOVE_TIMEOUT_MS) {
            motor2_set_state(MOTOR_STOP, MOTOR_SPEED); // CHANGE
            // Optionally, set an error flag here
            return; // Exit the function
        }

        HAL_Delay(LOOP_DELAY_MS);
    }
}

void moveMotor1ToADCValue_And_Measure_Smoothness(uint16_t targetADC, uint32_t expected_duration_ms, uint16_t tolerance, int32_t* max_error_out)
{
    const int32_t start_adc = Pot1_2[0];
    const int32_t end_adc = targetADC;
    int32_t error_to_target;
    *max_error_out = 0;
    const uint32_t move_start_tick = HAL_GetTick();

    while (1) {
        uint16_t current_adc = Pot1_2[0];
        error_to_target = end_adc - current_adc;
        if (labs(error_to_target) <= tolerance) {
            motor1_set_state(MOTOR_STOP, MOTOR_SPEED);
            return;
        }
        uint32_t elapsed_time = HAL_GetTick() - move_start_tick;
        if (expected_duration_ms > 0) {
            int32_t travel_distance = end_adc - start_adc;
            int32_t ideal_position = start_adc + (int32_t)(((long long)travel_distance * elapsed_time) / expected_duration_ms);
            int32_t current_smoothness_error = current_adc - ideal_position;
            if (labs(current_smoothness_error) > labs(*max_error_out)) {
                *max_error_out = current_smoothness_error;
            }
        }
        if (error_to_target > 0) {
            motor1_set_state(MOTOR_FORWARD, MOTOR_SPEED);
        } else {
            motor1_set_state(MOTOR_REVERSE, MOTOR_SPEED);
        }
        if (HAL_GetTick() - move_start_tick > MOVE_TIMEOUT_MS) { // <<< CHANGED
            motor1_set_state(MOTOR_STOP, MOTOR_SPEED);
            return;
        }
        HAL_Delay(LOOP_DELAY_MS); // <<< CHANGED
    }
}

void moveMotor2ToADCValue_And_Measure_Smoothness(uint16_t targetADC, uint32_t expected_duration_ms, uint16_t tolerance, int32_t* max_error_out)
{
    const int32_t start_adc = Pot1_2[1];
    const int32_t end_adc = targetADC;
    int32_t error_to_target;
    *max_error_out = 0;
    const uint32_t move_start_tick = HAL_GetTick();

    while (1) {
        uint16_t current_adc = Pot1_2[1];
        error_to_target = end_adc - current_adc;
        if (labs(error_to_target) <= tolerance) {
            motor2_set_state(MOTOR_STOP, MOTOR_SPEED);
            return;
        }
        uint32_t elapsed_time = HAL_GetTick() - move_start_tick;
        if (expected_duration_ms > 0) {
            int32_t travel_distance = end_adc - start_adc;
            int32_t ideal_position = start_adc + (int32_t)(((long long)travel_distance * elapsed_time) / expected_duration_ms);
            int32_t current_smoothness_error = current_adc - ideal_position;
            if (labs(current_smoothness_error) > labs(*max_error_out)) {
                *max_error_out = current_smoothness_error;
            }
        }
        if (error_to_target > 0) {
            motor2_set_state(MOTOR_FORWARD, MOTOR_SPEED);
        } else {
            motor2_set_state(MOTOR_REVERSE, MOTOR_SPEED);
        }
        if (HAL_GetTick() - move_start_tick > MOVE_TIMEOUT_MS) { // <<< CHANGED
            motor2_set_state(MOTOR_STOP, MOTOR_SPEED);
            return;
        }
        HAL_Delay(LOOP_DELAY_MS); // <<< CHANGED
    }
}

// ===================================================================
// 3. PUBLIC API FUNCTIONS (Declared in functions.h)
// ===================================================================

uint32_t ADC_MIN_TO_ADC_MAX_M1()
{
    float voltage_reading;

    moveMotor1ToADCValue(ADC_POS_MIN, MOVE_TOLERANCE_ADC);
    uint32_t Motor1_Start = HAL_GetTick();
    moveMotor1ToADCValue(ADC_POS_MAX, MOVE_TOLERANCE_ADC);
    uint32_t Motor1_End = HAL_GetTick();

    // <<< CORRECTED: Explicitly check for HAL_OK
    if (I2C_ReadVoltage(&hi2c1, Pot1, &voltage_reading) == HAL_OK)
    {
        parameters[PARAM_X1_MAX_POS_V] = voltage_reading;
    }

    parameters[PARAM_X1_P15V_I_MIN_MAX] = Max_PCurrent_M1;
    parameters[PARAM_X1_N15V_I_MIN_MAX] = Max_NCurrent_M1;
    //parameters[PARAM_X1_24V_I_MIN_MAX] = Max_PotCurrent;

    uint32_t time_taken = Motor1_End - Motor1_Start;
    moveMotor1ToADCValue(ADC_POS_HOME, MOVE_TOLERANCE_ADC);
    return time_taken;
}

uint32_t ADC_MAX_TO_ADC_MIN_M1()
{
    float voltage_reading;

    moveMotor1ToADCValue(ADC_POS_MAX, MOVE_TOLERANCE_ADC);
    uint32_t Motor1_Start = HAL_GetTick();
    moveMotor1ToADCValue(ADC_POS_MIN, MOVE_TOLERANCE_ADC);
    uint32_t Motor1_End = HAL_GetTick();

    // <<< CORRECTED: Explicitly check for HAL_OK
    if (I2C_ReadVoltage(&hi2c1, Pot1, &voltage_reading) == HAL_OK)
    {
        parameters[PARAM_X1_MIN_POS_V] = voltage_reading;
    }

    parameters[PARAM_X1_P15V_I_MAX_MIN] = Max_PCurrent_M1;
    parameters[PARAM_X1_N15V_I_MAX_MIN] = Max_NCurrent_M1;
    //parameters[PARAM_X1_24V_I_MAX_MIN] = Max_PotCurrent;

    uint32_t time_taken = Motor1_End - Motor1_Start;
    moveMotor1ToADCValue(ADC_POS_HOME, MOVE_TOLERANCE_ADC);
    return time_taken;
}

uint32_t ADC_MIN_TO_ADC_MAX_M2()
{
    float voltage_reading;

    moveMotor2ToADCValue(ADC_POS_MIN, MOVE_TOLERANCE_ADC);
    uint32_t Motor2_Start = HAL_GetTick();
    moveMotor2ToADCValue(ADC_POS_MAX, MOVE_TOLERANCE_ADC);
    uint32_t Motor2_End = HAL_GetTick();

    // <<< CORRECTED: Explicitly check for HAL_OK
    if (I2C_ReadVoltage(&hi2c1, Pot2, &voltage_reading) == HAL_OK)
    {
        parameters[PARAM_X2_MAX_POS_V] = voltage_reading;
    }

    parameters[PARAM_X2_P15V_I_MIN_MAX] = Max_PCurrent_M2;
    parameters[PARAM_X2_N15V_I_MIN_MAX] = Max_NCurrent_M2;
    //parameters[PARAM_X2_24V_I_MIN_MAX] = Max_PotCurrent_M2;

    uint32_t time_taken = Motor2_End - Motor2_Start;
    moveMotor2ToADCValue(ADC_POS_HOME, MOVE_TOLERANCE_ADC);
    return time_taken;
}

uint32_t ADC_MAX_TO_ADC_MIN_M2()
{
    float voltage_reading;

    moveMotor2ToADCValue(ADC_POS_MAX, MOVE_TOLERANCE_ADC);
    uint32_t Motor2_Start = HAL_GetTick();
    moveMotor2ToADCValue(ADC_POS_MIN, MOVE_TOLERANCE_ADC);
    uint32_t Motor2_End = HAL_GetTick();

    // <<< CORRECTED: Explicitly check for HAL_OK
    if (I2C_ReadVoltage(&hi2c1, Pot2, &voltage_reading) == HAL_OK)
    {
        parameters[PARAM_X2_MIN_POS_V] = voltage_reading;
    }

    parameters[PARAM_X2_P15V_I_MAX_MIN] = Max_PCurrent_M2;
    parameters[PARAM_X2_N15V_I_MAX_MIN] = Max_NCurrent_M2;
    //parameters[PARAM_X2_24V_I_MAX_MIN] = Max_PotCurrent_M2;

    uint32_t time_taken = Motor2_End - Motor2_Start;
    moveMotor2ToADCValue(ADC_POS_HOME, MOVE_TOLERANCE_ADC);
    return time_taken;
}

float Get_M1_Min_to_Max_Smoothness()
{
    uint32_t duration_ms = ADC_MIN_TO_ADC_MAX_M1();
    moveMotor1ToADCValue(ADC_POS_MIN, MOVE_TOLERANCE_ADC);
    HAL_Delay(SETTLE_DELAY_MS);
    int32_t max_error_in_adc;
    moveMotor1ToADCValue_And_Measure_Smoothness(ADC_POS_MAX, duration_ms, MOVE_TOLERANCE_ADC, &max_error_in_adc);
    float total_distance = (float)(ADC_POS_MAX - ADC_POS_MIN);
    if (total_distance == 0) return 0.0f;
    float normalized_error = (float)max_error_in_adc / total_distance;
    moveMotor1ToADCValue(ADC_POS_HOME, MOVE_TOLERANCE_ADC);
    return normalized_error;
}

float Get_M1_Max_to_Min_Smoothness(void)
{
    uint32_t duration_ms = ADC_MAX_TO_ADC_MIN_M1();
    moveMotor1ToADCValue(ADC_POS_MAX, MOVE_TOLERANCE_ADC);
    HAL_Delay(SETTLE_DELAY_MS);
    int32_t max_error_in_adc;
    moveMotor1ToADCValue_And_Measure_Smoothness(ADC_POS_MIN, duration_ms, MOVE_TOLERANCE_ADC, &max_error_in_adc);
    float total_distance = (float)(ADC_POS_MAX - ADC_POS_MIN);
    if (total_distance == 0) return 0.0f;
    float normalized_error = (float)max_error_in_adc / total_distance;
    moveMotor1ToADCValue(ADC_POS_HOME, MOVE_TOLERANCE_ADC);
    return normalized_error;
}

float Get_M2_Min_to_Max_Smoothness()
{
    uint32_t duration_ms = ADC_MIN_TO_ADC_MAX_M2();
    moveMotor2ToADCValue(ADC_POS_MIN, MOVE_TOLERANCE_ADC);
    HAL_Delay(SETTLE_DELAY_MS);
    int32_t max_error_in_adc;
    moveMotor2ToADCValue_And_Measure_Smoothness(ADC_POS_MAX, duration_ms, MOVE_TOLERANCE_ADC, &max_error_in_adc);
    float total_distance = (float)(ADC_POS_MAX - ADC_POS_MIN);
    if (total_distance == 0) return 0.0f;
    float normalized_error = (float)max_error_in_adc / total_distance;
    moveMotor2ToADCValue(ADC_POS_HOME, MOVE_TOLERANCE_ADC);
    return normalized_error;
}

float Get_M2_Max_to_Min_Smoothness(void)
{
    uint32_t duration_ms = ADC_MAX_TO_ADC_MIN_M2();
    moveMotor2ToADCValue(ADC_POS_MAX, MOVE_TOLERANCE_ADC);
    HAL_Delay(SETTLE_DELAY_MS);
    int32_t max_error_in_adc;
    moveMotor2ToADCValue_And_Measure_Smoothness(ADC_POS_MIN, duration_ms, MOVE_TOLERANCE_ADC, &max_error_in_adc);
    float total_distance = (float)(ADC_POS_MAX - ADC_POS_MIN);
    if (total_distance == 0) return 0.0f;
    float normalized_error = (float)max_error_in_adc / total_distance;
    moveMotor2ToADCValue(ADC_POS_HOME, MOVE_TOLERANCE_ADC);
    return normalized_error;
}
