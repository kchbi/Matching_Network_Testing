/*
 * functions.c
 *
 *  Created on: Jun 1, 2025
 *      Author: Aditya Singh
 */
#include "stm32f4xx_hal.h"     // Main HAL header (includes GPIO, TIM, etc.)
#include "main.h"
#include "functions.h"// Usually contains GPIO pin mappings and handles
#define MOTOR1_IN1 GPIO_PIN_7
#define MOTOR1_IN2 GPIO_PIN_8
#define MOTOR2_IN1 GPIO_PIN_9
#define MOTOR2_IN2 GPIO_PIN_10
uint32_t speed = 170;
uint32_t Potentiometer1_data[1];
uint32_t Potentiometer2_data[1];

void Motor1DriveDirection(uint32_t speed)
{
    HAL_GPIO_WritePin(GPIOE, MOTOR1_IN1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, MOTOR1_IN2, GPIO_PIN_RESET);
    TIM1->CCR1 = speed;
}

void Motor1DriveReverseDirection(uint32_t speed)
{
    HAL_GPIO_WritePin(GPIOE, MOTOR1_IN1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, MOTOR1_IN2, GPIO_PIN_SET);
    TIM1->CCR1 = speed;
}

void Motor2DriveDirection(uint32_t speed) {
    HAL_GPIO_WritePin(GPIOE, MOTOR2_IN1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, MOTOR2_IN2, GPIO_PIN_RESET);
    TIM2->CCR1 = speed;
}

void Motor2DriveReverseDirection(uint32_t speed)
{
    HAL_GPIO_WritePin(GPIOE, MOTOR2_IN1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, MOTOR2_IN2, GPIO_PIN_SET);
    TIM2->CCR1 = speed;
}

int Lag(void)
{
    // Start the motor
    MotorMotor1DriveDirection1(170);
    uint32_t start_time_motor = HAL_GetTick();

    // Store initial potentiometer position
    uint32_t init_position = Potentiometer1_data[0];

    // Wait for a change in potentiometer reading
    while ((Potentiometer1_data[0] - init_position) < 5) {
        // Optionally add a timeout mechanism to avoid infinite loop
        if ((HAL_GetTick() - start_time_motor) > TIMEOUT_VALUE) {
            return -1; // Indicate timeout
        }
    }

    uint32_t start_time_pot = HAL_GetTick();
    uint32_t lagtime = start_time_pot - start_time_motor;
    return lagtime;
}





