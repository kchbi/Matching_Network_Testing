/*
 * test.c
 *
 *  Created on: 20-Jan-2026
 *      Author: AdityaSingh
 */


#include "test.h"
#include "motor_control.h"
#include "i2ccomm.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/* ===== External application data ===== */



#define CAL_HOLD_PWM  5.0f



extern volatile uint8_t g_test_is_running;

/* These are still global measurement outputs */
extern float Max_PCurrent_M1;
extern float Max_PCurrent_M2;

float parameters[NUM_PARAMETERS] = {0.0f};


/* ===== Init ===== */

void Test_Init(void)
{
    /* Start RF stimulus */

    /* Scan I2C for diagnostics */
    I2C_Scan(&hi2c1);
    HAL_Delay(1000);

    /* INA226 initialisation */
    if (I2C_Init(&hi2c1, T15VP) != HAL_OK)
        printf("Failed to initialize +15V INA226\r\n");
    else
        printf(" +15V INA226 initialized\r\n");

    HAL_Delay(100);
    if (I2C_Init(&hi2c1, T15VN) != HAL_OK)
        printf("Failed to initialize -15V INA226\r\n");
    else
        printf(" -15V INA226 initialized\r\n");

    HAL_Delay(100);

    if (I2C_Init(&hi2c1, Pot1) != HAL_OK)
        printf("Failed to initialize Pot1 INA226\r\n");
    else
        printf("Pot1 INA226 initialized\r\n");

    HAL_Delay(100);

    if (I2C_Init(&hi2c1, Pot2) != HAL_OK)
        printf("Failed to initialize Pot2 INA226\r\n");
    else
        printf("Pot2 INA226 initialized\r\n");

    HAL_Delay(100);
}

static void Calibrate_Pot(Motor_ID_t motor, uint8_t is_max)
{
    float cmd = is_max ? CAL_HOLD_PWM : -CAL_HOLD_PWM;
    Motor_Set(motor, cmd);
}

void Stream_Pot_Value(Motor_ID_t motor, ParameterIndex_t param_index)
{
    float voltage = 0.0f;

    uint16_t pot_addr = (motor == MOTOR_1) ? Pot1 : Pot2;

    if (I2C_ReadVoltage(&hi2c1, pot_addr, &voltage) != HAL_OK)
        return;

    parameters[param_index] = voltage;

    printf("DATA,");
    for (int i = 0; i < 4; i++) {
        printf("%ld", (int32_t)(parameters[i] * 1000.0f));
        if (i < 3) printf(",");
    }
    printf("\n");
    fflush(stdout);
}




/* ===== Corner positioning ===== */
/* Maps from: g_test_is_running == 2..5 */

void Test_GotoCorner(uint8_t corner_id)
{
    switch (corner_id)
    {
        case STATE_CAL_M1_MIN:
            Calibrate_Pot(MOTOR_1, 0);
            Stream_Pot_Value(MOTOR_1,PARAM_X1_MIN_POS_V);
            break;

        case STATE_CAL_M1_MAX:
            Calibrate_Pot(MOTOR_1, 1);
            Stream_Pot_Value(MOTOR_1,PARAM_X1_MAX_POS_V);
            break;

        case STATE_CAL_M2_MIN:
            Calibrate_Pot(MOTOR_2, 0);
            Stream_Pot_Value(MOTOR_2,PARAM_X2_MIN_POS_V);
            break;

        case STATE_CAL_M2_MAX:
            Calibrate_Pot(MOTOR_2, 1);
            Stream_Pot_Value(MOTOR_2,PARAM_X2_MAX_POS_V);
            break;

        default:
            Motor_StopAll();
            break;
    }
}


void Stop_test(void)
{
	Motor_StopAll();

}
