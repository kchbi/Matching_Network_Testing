/*
 * motor_driver.c
 *
 *  Created on: 20-Jan-2026
 *      Author: AdityaSingh
 */


#include "motor_driver.h"
#include "main.h"

#define PWM_MAX_DUTY     200
#define PWM_MIN_DUTY     110

static float current_cmd[2] = {0.0f, 0.0f};
#define RAMP_STEP  5.0f      // PWM units per update


static void Motor_SetPins(Motor_ID_t motor, GPIO_PinState in1, GPIO_PinState in2)
{
    if (motor == MOTOR_1) {
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR1_IN1, in1);
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR1_IN2, in2);
    } else {
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR2_IN1, in1);
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR2_IN2, in2);
    }
}

static void Motor_SetPWM(Motor_ID_t motor, uint32_t pwm)
{
    if (motor == MOTOR_1) {
        TIM1->CCR1 = pwm;
    } else {
        TIM2->CCR1 = pwm;
    }
}

void Motor_Set(Motor_ID_t motor, float target_cmd)
{
    float *cmd = &current_cmd[motor];

    /* Ramp toward target */
    if (*cmd < target_cmd)
    {
        *cmd += RAMP_STEP;
        if (*cmd > target_cmd)
            *cmd = target_cmd;
    }
    else if (*cmd > target_cmd)
    {
        *cmd -= RAMP_STEP;
        if (*cmd < target_cmd)
            *cmd = target_cmd;
    }

    float command = *cmd;
    float abs_cmd;
    uint32_t pwm;

    /* Dead-zone */
    if (command > 1.0f)
    {
        Motor_SetPins(motor, GPIO_PIN_SET, GPIO_PIN_RESET);
        abs_cmd = command;
    }
    else if (command < -1.0f)
    {
        Motor_SetPins(motor, GPIO_PIN_RESET, GPIO_PIN_SET);
        abs_cmd = -command;
    }
    else
    {
        Motor_Stop(motor);
        return;
    }

    /* Clamp */
    if (abs_cmd > PWM_MAX_DUTY)
        abs_cmd = PWM_MAX_DUTY;

    pwm = (uint32_t)abs_cmd;


    Motor_SetPWM(motor, pwm);
}


void Motor_Stop(Motor_ID_t motor)
{
    Motor_SetPins(motor, GPIO_PIN_SET, GPIO_PIN_SET);
    Motor_SetPWM(motor, 0);
}

void Motor_StopAll(void)
{
    Motor_Stop(MOTOR_1);
    Motor_Stop(MOTOR_2);
}
