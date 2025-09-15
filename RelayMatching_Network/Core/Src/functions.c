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

uint16_t ADC_POS_MIN_M1 = 530;
uint16_t ADC_POS_MAX_M1 = 2230;
uint16_t ADC_POS_HOME_M1 = 0;



uint16_t ADC_POS_MIN_M2 = 530;
uint16_t ADC_POS_MAX_M2 = 2230;
uint16_t ADC_POS_HOME_M2 = 0;

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
 * @brief  Finds the maximum position for Motor 1 and updates the ADC_POS_MAX_M1 global variable.
 * @retval The voltage at the maximum position, or -1.0f on timeout.
 */
float FindMaxPositionVoltageMotor1()
{
    uint16_t lastADC;
    uint16_t currentADC;
    uint16_t tolerance = 5;
    float voltage_reading = -1.0f;
    uint32_t start_tick = HAL_GetTick();

    lastADC = Pot1_2[0];
    motor1_set_state(MOTOR_FORWARD, MOTOR_SPEED);

    while (1)
    {
        HAL_Delay(100);
        currentADC = Pot1_2[0];

        if (abs(currentADC - lastADC) <= tolerance)
        {
            ADC_POS_MAX_M1 = currentADC; // Update with correct variable name
            I2C_ReadVoltage(&hi2c1, Pot1, &voltage_reading);
            break;
        }

        lastADC = currentADC;

        if ((HAL_GetTick() - start_tick) > MOVE_TIMEOUT_MS)
        {
            break;
        }
    }

    motor1_set_state(MOTOR_STOP, 0);
    return voltage_reading;
}
/**
 * @brief  Finds the maximum position for Motor 2 and updates the ADC_POS_MAX_M2 global variable.
 * @retval The voltage at the maximum position, or -1.0f on timeout.
 */
float FindMaxPositionVoltageMotor2()
{
    uint16_t lastADC;
    uint16_t currentADC;
    uint16_t tolerance = 5;
    float voltage_reading = -1.0f;
    uint32_t start_tick = HAL_GetTick();

    lastADC = Pot1_2[1];
    motor2_set_state(MOTOR_FORWARD, MOTOR_SPEED);

    while (1)
    {
        HAL_Delay(100);
        currentADC = Pot1_2[1];

        if (abs(currentADC - lastADC) <= tolerance)
        {
            ADC_POS_MAX_M2 = currentADC; // Update with correct variable name
            I2C_ReadVoltage(&hi2c1, Pot2, &voltage_reading);
            break;
        }

        lastADC = currentADC;

        if ((HAL_GetTick() - start_tick) > MOVE_TIMEOUT_MS)
        {
            break;
        }
    }

    motor2_set_state(MOTOR_STOP, 0);
    return voltage_reading;
}
void Update_Home_Positions(void)
{
    ADC_POS_HOME_M1 = (ADC_POS_MIN_M1 + ADC_POS_MAX_M1) / 2;
    ADC_POS_HOME_M2 = (ADC_POS_MIN_M2 + ADC_POS_MAX_M2) / 2;
}

/**
 * @brief  Finds the minimum position for Motor 2 and updates the ADC_POS_MIN_M2 global variable.
 * @retval The voltage at the minimum position, or -1.0f on timeout.
 */
float FindMinPositionVoltageMotor2()
{
    uint16_t lastADC;
    uint16_t currentADC;
    uint16_t tolerance = 5;
    float voltage_reading = -1.0f;
    uint32_t start_tick = HAL_GetTick();

    lastADC = Pot1_2[1];
    motor2_set_state(MOTOR_REVERSE, MOTOR_SPEED);

    while (1)
    {
        HAL_Delay(100);
        currentADC = Pot1_2[1];

        if (abs(currentADC - lastADC) <= tolerance)
        {
            ADC_POS_MIN_M2 = currentADC; // Update with correct variable name
            I2C_ReadVoltage(&hi2c1, Pot2, &voltage_reading);
            break;
        }

        lastADC = currentADC;

        if ((HAL_GetTick() - start_tick) > MOVE_TIMEOUT_MS)
        {
            break;
        }
    }

    motor2_set_state(MOTOR_STOP, 0);
    return voltage_reading;
}

/**
 * @brief  Finds the minimum position for Motor 1 and updates the ADC_POS_MIN_M1 global variable.
 * @retval The voltage at the minimum position, or -1.0f on timeout.
 */
float FindMinPositionVoltageMotor1()
{
    uint16_t lastADC;
    uint16_t currentADC;
    uint16_t tolerance = 5;
    float voltage_reading = -1.0f;
    uint32_t start_tick = HAL_GetTick();

    lastADC = Pot1_2[0];
    motor1_set_state(MOTOR_REVERSE, MOTOR_SPEED);

    while (1)
    {
        HAL_Delay(100);
        currentADC = Pot1_2[0];

        if (abs(currentADC - lastADC) <= tolerance)
        {
            ADC_POS_MIN_M1 = currentADC; // Update with correct variable name
            I2C_ReadVoltage(&hi2c1, Pot1, &voltage_reading);
            break;
        }

        lastADC = currentADC;

        if ((HAL_GetTick() - start_tick) > MOVE_TIMEOUT_MS)
        {
            break;
        }
    }

    motor1_set_state(MOTOR_STOP, 0);
    return voltage_reading;
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
    float Inst_Current;

    // Reset the global variables for Motor 1 before starting the move
    Max_PCurrent_M1 = 0;
//    Max_NCurrent_M1 = 0;
//    Voltage_M1 = 0; // Assuming you have this variable as well

    while (1) {
        // --- FOR MOTOR 1: Read the potentiometer for Motor 1 ---
        currentADC = Pot1_2[0];
        error = (int32_t)targetADC - (int32_t)currentADC;

        // --- FOR MOTOR 1: Read positive current and update max value ---
        I2C_ReadCurrent(&hi2c1, Motor1, &Inst_Current);

        // --- CORRECTED: Check the local variable 'Inst_CurrentP', not 'Inst_Current' ---
        float Abs_Inst_Current = fabs(Inst_Current);
            if (Abs_Inst_Current > Max_PCurrent_M1)
            {
                Max_PCurrent_M1 = Abs_Inst_Current;
            }


//        // --- FOR MOTOR 1: Read negative current and update max value ---
//        I2C_ReadCurrent(&hi2c1, Motor1N, &Inst_CurrentN);
//
//        // --- CORRECTED: Check 'Inst_CurrentN' and update 'Max_NCurrent_M1' ---
//        if (Inst_CurrentN > Max_NCurrent_M1)
//        {
//            Max_NCurrent_M1 = Inst_CurrentN; // This now updates the correct variable
//        }

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
    float Inst_Current;

    // Reset the global variables for Motor 2 before starting the move
    Max_PCurrent_M2 = 0;
    // Max_NCurrent_M2 = 0; // Assuming you have these for Motor 2
    // Voltage_M2 = 0;

    while (1) {
        // --- FOR MOTOR 2: Read the potentiometer for Motor 2 ---
        // Assuming Motor 2's pot is at index 1 of the array
        currentADC = Pot1_2[1];
        error = (int32_t)targetADC - (int32_t)currentADC;

        // --- FOR MOTOR 2: Read current and update max value ---
        // Assuming 'Motor2' is the I2C address #define for the second sensor
        I2C_ReadCurrent(&hi2c1, Motor2, &Inst_Current);

        // Calculate absolute current and update the max value for Motor 2
        float Abs_Inst_Current = fabs(Inst_Current);
        if (Abs_Inst_Current > Max_PCurrent_M2)
        {
            Max_PCurrent_M2 = Abs_Inst_Current;
        }

        // Check if the motor is at the target position
        if (labs(error) <= tolerance) {
            motor2_set_state(MOTOR_STOP, 0); // Use motor 2's control function
            return; // Exit the function, move complete
        }

        // Move the motor in the correct direction
        if (error > 0) {
            // Target is greater than current, motor needs to move to increase ADC
            motor2_set_state(MOTOR_FORWARD, MOTOR_SPEED); // Use motor 2's control function
        } else {
            // Target is less than current, motor needs to move to decrease ADC
            motor2_set_state(MOTOR_REVERSE, MOTOR_SPEED); // Use motor 2's control function
        }

        // Check for timeout to prevent the motor from running forever
        if (HAL_GetTick() - timeout_start > MOVE_TIMEOUT_MS) {
            motor2_set_state(MOTOR_STOP, 0); // Use motor 2's control function
            // You might want to set an error flag here to indicate a timeout occurred
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
    // float voltage_reading; // <<< REMOVED

    moveMotor1ToADCValue(ADC_POS_MIN_M1, MOVE_TOLERANCE_ADC);
    uint32_t Motor1_Start = HAL_GetTick();
    moveMotor1ToADCValue(ADC_POS_MAX_M1, MOVE_TOLERANCE_ADC);
    uint32_t Motor1_End = HAL_GetTick();
    parameters[PARAM_X1_P15V_I_MIN_MAX] = Max_PCurrent_M1;

    uint32_t time_taken = Motor1_End - Motor1_Start;
    moveMotor1ToADCValue(ADC_POS_HOME_M1, MOVE_TOLERANCE_ADC);
    return time_taken;
}

uint32_t ADC_MAX_TO_ADC_MIN_M1()
{
    // float voltage_reading; // <<< REMOVED

    moveMotor1ToADCValue(ADC_POS_MAX_M1, MOVE_TOLERANCE_ADC);
    uint32_t Motor1_Start = HAL_GetTick();
    moveMotor1ToADCValue(ADC_POS_MIN_M1, MOVE_TOLERANCE_ADC);
    uint32_t Motor1_End = HAL_GetTick();

    parameters[PARAM_X1_P15V_I_MAX_MIN] = Max_PCurrent_M1;

    uint32_t time_taken = Motor1_End - Motor1_Start;
    moveMotor1ToADCValue(ADC_POS_HOME_M1, MOVE_TOLERANCE_ADC);
    return time_taken;
}

float Get_M1_Min_to_Max_Smoothness()
{
    uint32_t duration_ms = ADC_MIN_TO_ADC_MAX_M1();
    moveMotor1ToADCValue(ADC_POS_MIN_M1, MOVE_TOLERANCE_ADC);
    HAL_Delay(SETTLE_DELAY_MS);
    int32_t max_error_in_adc;
    moveMotor1ToADCValue_And_Measure_Smoothness(ADC_POS_MAX_M1, duration_ms, MOVE_TOLERANCE_ADC, &max_error_in_adc);
    float total_distance = (float)(ADC_POS_MAX_M1 - ADC_POS_MIN_M1);
    if (total_distance == 0) return 0.0f;
    float normalized_error = (float)max_error_in_adc / total_distance;
    moveMotor1ToADCValue(ADC_POS_HOME_M1, MOVE_TOLERANCE_ADC);
    return normalized_error;
}

float Get_M1_Max_to_Min_Smoothness(void)
{
    uint32_t duration_ms = ADC_MAX_TO_ADC_MIN_M1();
    moveMotor1ToADCValue(ADC_POS_MAX_M1, MOVE_TOLERANCE_ADC);
    HAL_Delay(SETTLE_DELAY_MS);
    int32_t max_error_in_adc;
    moveMotor1ToADCValue_And_Measure_Smoothness(ADC_POS_MIN_M1, duration_ms, MOVE_TOLERANCE_ADC, &max_error_in_adc);
    float total_distance = (float)(ADC_POS_MAX_M1 - ADC_POS_MIN_M1);
    if (total_distance == 0) return 0.0f;
    float normalized_error = (float)max_error_in_adc / total_distance;
    moveMotor1ToADCValue(ADC_POS_HOME_M1, MOVE_TOLERANCE_ADC);
    return normalized_error;
}

uint32_t ADC_MIN_TO_ADC_MAX_M2()
{
    // float voltage_reading; // <<< REMOVED

    moveMotor2ToADCValue(ADC_POS_MIN_M2, MOVE_TOLERANCE_ADC);
    uint32_t Motor2_Start = HAL_GetTick();
    moveMotor2ToADCValue(ADC_POS_MAX_M2, MOVE_TOLERANCE_ADC);
    uint32_t Motor2_End = HAL_GetTick();

    parameters[PARAM_X2_P15V_I_MIN_MAX] = Max_PCurrent_M2;

    uint32_t time_taken = Motor2_End - Motor2_Start;
    moveMotor2ToADCValue(ADC_POS_HOME_M2, MOVE_TOLERANCE_ADC);
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
    moveMotor2ToADCValue(ADC_POS_HOME_M2, MOVE_TOLERANCE_ADC);
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
    moveMotor2ToADCValue(ADC_POS_HOME_M2, MOVE_TOLERANCE_ADC);
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
    moveMotor2ToADCValue(ADC_POS_HOME_M2, MOVE_TOLERANCE_ADC);
    return normalized_error;
}
