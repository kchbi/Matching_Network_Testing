/*
 * motor_driver.c
 *
 *  Created on: 20-Jan-2026
 *      Author: AdityaSingh
 */


#include "motor_driver.h"
#include "main.h"
#include "math.h"

#define PWM_MAX_DUTY     200
#define PWM_MIN_DUTY     110

#define RAMP_STEP  5.0f      // PWM units per update
static float target_cmd[2] = {0.0f, 0.0f};
static float current_cmd[2] = {0.0f, 0.0f};

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

void Motor_Set(Motor_ID_t motor, float cmd)
{
    /* Clamp to allowed range */
    if (cmd > PWM_MAX_DUTY)  cmd = PWM_MAX_DUTY;
    if (cmd < -PWM_MAX_DUTY) cmd = -PWM_MAX_DUTY;

    target_cmd[motor] = cmd;
}

void Motor_RampUpdate(Motor_ID_t motor)
{
    float *cur = &current_cmd[motor];
    float tgt = target_cmd[motor];

    /* === Direction safe switching === */
    if ((*cur > 0 && tgt < 0) || (*cur < 0 && tgt > 0))
    {
        /* First ramp to zero before reversing */
        if (*cur > 0)
            *cur -= RAMP_STEP;
        else
            *cur += RAMP_STEP;

        if (fabsf(*cur) < RAMP_STEP)
            *cur = 0;

    }
    else
    {
        /* Normal ramp toward target */
        if (*cur < tgt)
        {
            *cur += RAMP_STEP;
            if (*cur > tgt) *cur = tgt;
        }
        else if (*cur > tgt)
        {
            *cur -= RAMP_STEP;
            if (*cur < tgt) *cur = tgt;
        }
    }

    /* === Apply output === */
    float command = *cur;
    float abs_cmd;
    uint32_t pwm;

    if (fabsf(command) < 1.0f)
    {
        Motor_Stop(motor);
        return;
    }

    if (command > 0)
    {
        Motor_SetPins(motor, GPIO_PIN_SET, GPIO_PIN_RESET);
        abs_cmd = command;
    }
    else
    {
        Motor_SetPins(motor, GPIO_PIN_RESET, GPIO_PIN_SET);
        abs_cmd = -command;
    }

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


