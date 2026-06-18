/*
 * motor_driver.c
 *
 *  Created on: 20-Jan-2026
 *      Author: AdityaSingh
 */

#include "motor_driver.h"
#include "main.h"
#include "DAC_Generator.h"



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
void Motor_Set(Motor_ID_t motor, uint16_t command)
{
    if(command > 4095)
        command = 4095;

    switch(motor)
    {
        case MOTOR_1:
            DACGenerator_Start(&hdac, DAC_CHANNEL_1, command);
            break;

        case MOTOR_2:
            DACGenerator_Start(&hdac, DAC_CHANNEL_2, command);
            break;

        default:
            break;
    }
}
/* ============================================================
 * Motor_Stop — Hard brake (both half-bridges shorted high)
 *
 * Use this only at:
 *   - End of a move (after target reached)
 *   - Emergency stop
 *   - Calibration end
 * ============================================================ */
void Motor_Stop(Motor_ID_t motor)
{
    switch(motor)
    {
        case MOTOR_1:
            DACGenerator_Stop(&hdac, DAC_CHANNEL_1);
            break;

        case MOTOR_2:
            DACGenerator_Stop(&hdac, DAC_CHANNEL_2);
            break;

        default:
            break;
    }
}

void Motor_StopAll(void)
{
    DACGenerator_Stop(&hdac, DAC_CHANNEL_1);
    DACGenerator_Stop(&hdac, DAC_CHANNEL_2);
}
