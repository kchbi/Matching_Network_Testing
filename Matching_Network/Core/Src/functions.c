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
#define MOTOR2_IN2          GPIO_PIN_10

#define MOTOR_SPEED         (200)     //CHANGED: Centralized speed setting
#define PWM_MAX_DUTY        (200)     //CHANGED: Matched to timer period for full range
#define MOVE_TIMEOUT_MS     (5000)    //CHANGED: Timeout for any motor movement
#define SETTLE_DELAY_MS     (100)     //CHANGED: Delay for motor to settle
#define LOOP_DELAY_MS       (1)      //Delay in motor control loops

uint16_t ADC_POS_MIN_M1 = 530;
uint16_t ADC_POS_MAX_M1 = 2230;
uint16_t ADC_POS_HOME_M1 = 0;

float Kp = 0.015f;
float Ki = 0.002f;
float Kd = 0.000f;

int32_t PWM_MIN_DUTY = 110 ;        //Might Change or might need to Find this with a function.
uint16_t ADC_POS_MIN_M2 = 530;
uint16_t ADC_POS_MAX_M2 = 2230;
uint16_t ADC_POS_HOME_M2 = 0;

extern I2C_HandleTypeDef hi2c1;


void motor1_set_state(float speed)
{
    uint32_t command;
    float abs_speed;

    // 1. Deadzone Protection: Treat tiny noise as 0
    if (speed > 0.01f) {
        // --- FORWARD ---
        HAL_GPIO_WritePin(GPIOE, MOTOR1_IN1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, MOTOR1_IN2, GPIO_PIN_RESET);

        abs_speed = speed;
    }
    else if (speed < -0.01f) {
        // --- REVERSE ---
        HAL_GPIO_WritePin(GPIOE, MOTOR1_IN1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, MOTOR1_IN2, GPIO_PIN_SET);

        abs_speed = -speed; // Make positive for calculation
    }
    else {
        // --- STOP ---
        // Force motor terminals to same state (Brake) or float (Coast)
        // Your code sets both SET -> Brake High
        HAL_GPIO_WritePin(GPIOE, MOTOR1_IN1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, MOTOR1_IN2, GPIO_PIN_SET);
        TIM1->CCR1 = 110;
        return;
    }

    // 2. Clamp Logic
    if (abs_speed > (float)PWM_MAX_DUTY) {
        abs_speed = (float)PWM_MAX_DUTY;
    }

    // 3. Deadband Scaling Math
    // Map 0.01 -> 200.0  TO  110 -> 200
    float output_range = (float)(PWM_MAX_DUTY - PWM_MIN_DUTY);

    // Using abs_speed here ensures the math is clean
    float scaled_output = (float)PWM_MIN_DUTY + (abs_speed / (float)PWM_MAX_DUTY) * output_range;

    command = (uint32_t)scaled_output;
    TIM1->CCR1 = command;
}



void motor2_set_state(float speed)
{
    uint32_t command;
    float abs_speed;

    // 1. Deadzone Protection: Treat tiny noise (< 0.01) as 0
    if (speed > 0.01f) {
        // --- FORWARD ---
        HAL_GPIO_WritePin(GPIOE, MOTOR2_IN1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, MOTOR2_IN2, GPIO_PIN_RESET);

        abs_speed = speed;
    }
    else if (speed < -0.01f) {
        // --- REVERSE ---
        HAL_GPIO_WritePin(GPIOE, MOTOR2_IN1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, MOTOR2_IN2, GPIO_PIN_SET);

        abs_speed = -speed; // Make positive for calculation
    }
    else {
        // --- STOP ---
        // Brake High (both pins SET) is usually preferred for holding position
        HAL_GPIO_WritePin(GPIOE, MOTOR2_IN1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, MOTOR2_IN2, GPIO_PIN_SET);
        TIM2->CCR1 = 110; // Using Timer 2 for Motor 2
        return;
    }

    // 2. Clamp Logic
    if (abs_speed > (float)PWM_MAX_DUTY) {
        abs_speed = (float)PWM_MAX_DUTY;
    }

    // 3. Deadband Scaling Math
    // This ensures that even a speed of 0.5 results in PWM ~110
    float output_range = (float)(PWM_MAX_DUTY - PWM_MIN_DUTY);
    float scaled_output = (float)PWM_MIN_DUTY + (abs_speed / (float)PWM_MAX_DUTY) * output_range;

    command = (uint32_t)scaled_output;
    TIM2->CCR1 = command;
}


float FindMaxPositionVoltageMotor1()
{
    uint16_t lastADC;
    uint16_t currentADC;
    uint16_t tolerance = 5;
    float voltage_reading = -1.0f;
    uint32_t start_tick = HAL_GetTick();

    lastADC = Pot1_2[0];
    motor1_set_state(5);

    while (1)
    {
        HAL_Delay(100);
       // osDelay(100); For Future Purposes
        currentADC = Pot1_2[0];

        if (abs(currentADC - lastADC) <= tolerance)
        {
            motor1_set_state(0);
            ADC_POS_MAX_M1 = currentADC; // Update with correct variable name
            I2C_ReadVoltage(&hi2c1, Pot1, &voltage_reading);
            break;
        }

        lastADC = currentADC;

        if ((HAL_GetTick() - start_tick) > MOVE_TIMEOUT_MS)
        {
            motor1_set_state(0);
            break;
        }
    }

    return voltage_reading;
}


float FindMaxPositionVoltageMotor2()
{
    uint16_t lastADC;
    uint16_t currentADC;
    uint16_t tolerance = 5;
    float voltage_reading = -1.0f;
    uint32_t start_tick = HAL_GetTick();

    lastADC = Pot1_2[1];
    motor2_set_state(5);

    while (1)
    {
        HAL_Delay(100);
        currentADC = Pot1_2[1];

        if (abs(currentADC - lastADC) <= tolerance)
        {
            motor2_set_state(0);
            ADC_POS_MAX_M2 = currentADC; // Update with correct variable name
            I2C_ReadVoltage(&hi2c1, Pot2, &voltage_reading);
            break;
        }

        lastADC = currentADC;

        if ((HAL_GetTick() - start_tick) > MOVE_TIMEOUT_MS)
        {
            motor2_set_state(0);
            break;
        }
    }


    return voltage_reading;
}
void Update_Home_Positions(void)
{
    ADC_POS_HOME_M1 = (ADC_POS_MIN_M1 + ADC_POS_MAX_M1) / 2;
    ADC_POS_HOME_M2 = (ADC_POS_MIN_M2 + ADC_POS_MAX_M2) / 2;
}

float FindMinPositionVoltageMotor2()
{
    uint16_t lastADC;
    uint16_t currentADC;
    uint16_t tolerance = 5;
    float voltage_reading = -1.0f;
    uint32_t start_tick = HAL_GetTick();

    lastADC = Pot1_2[1];
    motor2_set_state(-5);

    while (1)
    {
        HAL_Delay(100);
        currentADC = Pot1_2[1];

        if (abs(currentADC - lastADC) <= tolerance)
        {
            motor2_set_state(0);
            ADC_POS_MIN_M2 = currentADC; // Update with correct variable name
            I2C_ReadVoltage(&hi2c1, Pot2, &voltage_reading);
            break;
        }

        lastADC = currentADC;

        if ((HAL_GetTick() - start_tick) > MOVE_TIMEOUT_MS)
        {
            motor2_set_state(0);
            break;
        }
    }


    return voltage_reading;
}


float FindMinPositionVoltageMotor1()
{
    uint16_t lastADC;
    uint16_t currentADC;
    uint16_t tolerance = 5;
    float voltage_reading = -1.0f;
    uint32_t start_tick = HAL_GetTick();

    lastADC = Pot1_2[0];
    motor1_set_state(-5);

    while (1)
    {
        HAL_Delay(100);
        currentADC = Pot1_2[0];

        if (abs(currentADC - lastADC) <= tolerance)
        {
            motor1_set_state(0);
            ADC_POS_MIN_M1 = currentADC; // Update with correct variable name
            I2C_ReadVoltage(&hi2c1, Pot1, &voltage_reading);
            break;
        }

        lastADC = currentADC;

        if ((HAL_GetTick() - start_tick) > MOVE_TIMEOUT_MS)
        {
            motor1_set_state(0);
            break;
        }
    }

    return voltage_reading;
}

float PID_Control(float Kp, float Ki, float Kd,
                  float error, uint32_t prev_tick,
                  float currentADC, uint8_t reset)
{
    static float previousADC = 0.0f;
    static float integral = 0.0f;

    if (reset) {
        previousADC = currentADC;
        integral = 0.0f;
        return 0.0f;  // Output 0 on reset
    }



    float dt = (HAL_GetTick() - prev_tick) / 1000.0f;  // convert to seconds
    if (dt <= 0.0f) dt = 0.001f; // avoid divide-by-zero

    // --- Proportional ---
    float P = Kp * error;

    // --- Integral ---
    integral += error * dt;
    float I = Ki * integral;

    // --- Derivative (on measurement) ---
    float ds = (currentADC - previousADC) / dt; // rate of change of feedback
    float D = -Kd * ds; // negative to resist rapid motion

    // --- Combine ---
    float PID = P + I + D;

    // Update for next iteration
    previousADC = currentADC;

    return PID;
}


void moveMotor1ToADCValue(uint16_t targetADC, uint16_t tolerance)
{
    uint16_t currentADC = 0;
    int32_t error = 0;
    uint32_t timeout_start = HAL_GetTick();
    uint32_t prev_tick = HAL_GetTick();
    float Inst_Current;
    Max_PCurrent_M1 = 0;
    PID_Control(Kp, Ki, Kd, error, prev_tick, currentADC,1);

    while (1) {



        // --- Read current for monitoring ---
        I2C_ReadCurrent(&hi2c1, Motor1, &Inst_Current);
        float Abs_Inst_Current = fabs(Inst_Current);
        if (Abs_Inst_Current > Max_PCurrent_M1)
            Max_PCurrent_M1 = Abs_Inst_Current;
        // --- Read current position ---
        currentADC = Pot1_2[0];
        error = (int32_t)targetADC - (int32_t)currentADC;

        // --- Check if target reached ---
        if (labs(error) <= tolerance) {

            motor1_set_state(0);
            return;
        }

        // --- Run PID ---
        float PIDOutput = PID_Control(Kp, Ki, Kd, error, prev_tick, currentADC,0);
        prev_tick = HAL_GetTick();

        // --- Command the motor ---
        motor1_set_state(PIDOutput);

        // --- Timeout protection ---
        if (HAL_GetTick() - timeout_start > MOVE_TIMEOUT_MS) {

            motor1_set_state(0);
            return;
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
    uint16_t currentADC = 0;
    int32_t error = 0;
    uint32_t timeout_start = HAL_GetTick();
    uint32_t prev_tick = HAL_GetTick();
    float Inst_Current;
    Max_PCurrent_M2 = 0;
    PID_Control(Kp, Ki, Kd, error, prev_tick, currentADC,1);


    while (1) {

        I2C_ReadCurrent(&hi2c1, Motor2, &Inst_Current);
        // Calculate absolute current and update the max value for Motor 2
        float Abs_Inst_Current = fabs(Inst_Current);
        if (Abs_Inst_Current > Max_PCurrent_M2)
        {
            Max_PCurrent_M2 = Abs_Inst_Current;
        }
        currentADC = Pot1_2[1];
        error = (int32_t)targetADC - (int32_t)currentADC;

        // Check if the motor is at the target position
        if (labs(error) <= tolerance) {
            motor2_set_state(0);
            return; // Exit the function, move complete
        }
        // --- Run PID ---
         float PIDOutput = PID_Control(Kp, Ki, Kd, error, prev_tick, currentADC,0);
         prev_tick = HAL_GetTick();

         // --- Command the motor ---
         motor2_set_state(PIDOutput);
        // Move the motor in the correct direction


        // Check for timeout to prevent the motor from running forever
        if (HAL_GetTick() - timeout_start > MOVE_TIMEOUT_MS) {
//            motor2_set_state(MOTOR_STOP, 0); // Use motor 2's control function
            motor2_set_state(0);
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
    uint32_t prev_tick = HAL_GetTick(); // For PID's dt calculation

    // Reset the PID controller's shared static variables before starting the move
    // Note: Passing uninitialized error/currentADC is safe ONLY because of the `if(reset)` check.
    uint16_t current_adc = 0; // Initialize to satisfy compiler warnings
    error_to_target = 0;      // Initialize to satisfy compiler warnings
    PID_Control(Kp, Ki, Kd, error_to_target, prev_tick, current_adc, 1);

    while (1) {
        // --- SENSE ---
        current_adc = Pot1_2[0];
        error_to_target = end_adc - current_adc;

        // --- CHECK FOR COMPLETION ---
        if (labs(error_to_target) <= tolerance) {
            motor1_set_state(0);
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

        // --- THINK (Calculate PID output) ---
        float PIDOutput = PID_Control(Kp, Ki, Kd, error_to_target, prev_tick, current_adc, 0);
        prev_tick = HAL_GetTick();

        // --- ACT ---
        motor1_set_state(PIDOutput);

        // --- TIMEOUT PROTECTION ---
        if (HAL_GetTick() - move_start_tick > MOVE_TIMEOUT_MS) {
            motor1_set_state(0);
            return;
        }
        HAL_Delay(LOOP_DELAY_MS);
    }
}
void moveMotor2ToADCValue_And_Measure_Smoothness(uint16_t targetADC, uint32_t expected_duration_ms, uint16_t tolerance, int32_t* max_error_out)
{
    const int32_t start_adc = Pot1_2[1];
    const int32_t end_adc = targetADC;
    int32_t error_to_target;
    *max_error_out = 0;

    const uint32_t move_start_tick = HAL_GetTick();
    uint32_t prev_tick = HAL_GetTick();

    // Reset the PID controller's shared static variables
    uint16_t current_adc = 0;
    error_to_target = 0;
    PID_Control(Kp, Ki, Kd, error_to_target, prev_tick, current_adc, 1);

    while (1) {
        // --- SENSE ---
        current_adc = Pot1_2[1];
        error_to_target = end_adc - current_adc;

        // --- CHECK FOR COMPLETION ---
        if (labs(error_to_target) <= tolerance) {
            motor2_set_state(0);
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

        // --- THINK (Calculate PID output) ---
        float PIDOutput = PID_Control(Kp, Ki, Kd, error_to_target, prev_tick, current_adc, 0);
        prev_tick = HAL_GetTick();

        // --- ACT ---
        motor2_set_state(PIDOutput);

        // --- TIMEOUT PROTECTION ---
        if (HAL_GetTick() - move_start_tick > MOVE_TIMEOUT_MS) {
            motor2_set_state(0);
            return;
        }
        HAL_Delay(LOOP_DELAY_MS);
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
    moveMotor1ToADCValue(ADC_POS_MIN_M1, MOVE_TOLERANCE_ADC);
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
    moveMotor1ToADCValue(ADC_POS_MIN_M1, MOVE_TOLERANCE_ADC);
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
    moveMotor1ToADCValue(ADC_POS_MIN_M1, MOVE_TOLERANCE_ADC);
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
    moveMotor1ToADCValue(ADC_POS_MIN_M1, MOVE_TOLERANCE_ADC);
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
