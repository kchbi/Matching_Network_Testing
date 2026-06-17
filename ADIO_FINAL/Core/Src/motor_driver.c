/*
 * motor_driver.c
 *
 *  Created on: 20-Jan-2026
 *      Author: AdityaSingh
 */

#include "motor_driver.h"
#include "main.h"


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
        TIM2->CCR1 = pwm;
    } else {
        TIM1->CCR1 = pwm;
    }
}


/* ============================================================
 * Motor_Set — Drives motor with signed command value
 *
 *   command  >  +0.5  → forward at |command| PWM (clamped)
 *   command  <  -0.5  → reverse at |command| PWM (clamped)
 *   |command| ≤ 0.5   → coast (free-wheel, NOT brake)
 *
 * NOTE: We do NOT brake here. Braking inside PID causes motor
 * to "slam" near target and lose smooth approach. Use
 * Motor_Stop() explicitly only when you want a full hard stop.
 * ============================================================ */
void Motor_Set(Motor_ID_t motor, float command)
{
	DACGenerator_Start(hdac1, channel, command,vref );



}


/* ============================================================
 * Motor_Stop — Hard brake (both half-bridges shorted high)
 *
 * Use this only at:
 *   - End of a move (after target reached)
 *   - Emergency stop
 *   - Calibration end
 *
 * Do NOT call this from inside PID — it fights the controller.
 * ============================================================ */
void Motor_Stop(Motor_ID_t motor)
{
    Motor_SetPins(motor, GPIO_PIN_SET, GPIO_PIN_SET);  /* both HIGH = brake */
    Motor_SetPWM(motor, PWM_MAX_DUTY);                  /* full PWM = strong brake */
}

void Motor_StopAll(void)
{
    Motor_Stop(MOTOR_1);
    Motor_Stop(MOTOR_2);
}
