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
#include "sine_generator.h"

/* ===== External application data ===== */

extern float parameters[];
extern volatile uint8_t g_test_is_running;

/* These are still global measurement outputs */
extern float Max_PCurrent_M1;
extern float Max_PCurrent_M2;

float parameters[NUM_PARAMETERS] = {0.0f};


/* ===== Init ===== */

void Test_Init(void)
{
    /* Start RF stimulus */
    SineGenerator_Start(&hdac, &htim6);

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


/* ===== Full automatic test ===== */
/* Maps from: g_test_is_running == 0 block */

void Test_Run(void)
{
    memset(parameters, 0, sizeof(parameters));

    /* -------- Calibration -------- */
    parameters[PARAM_X1_MIN_POS_V] = MotorControl_FindMin(MOTOR_1);
    HAL_Delay(200);
    parameters[PARAM_X1_MAX_POS_V] = MotorControl_FindMax(MOTOR_1);
    HAL_Delay(200);

    parameters[PARAM_X2_MIN_POS_V] = MotorControl_FindMin(MOTOR_2);
    HAL_Delay(200);
    parameters[PARAM_X2_MAX_POS_V] = MotorControl_FindMax(MOTOR_2);
    HAL_Delay(200);

    MotorControl_UpdateHome();

    /* -------- Timing tests -------- */
    parameters[PARAM_X1_TIME_MIN_MAX] =
        MotorControl_MinToMax(
            MOTOR_1,
            &parameters[PARAM_X1_P15V_I_MIN_MAX],
            &parameters[PARAM_X1_N15V_I_MIN_MAX]
        ) / 1000.0f;
    HAL_Delay(200);

    parameters[PARAM_X1_TIME_MAX_MIN] =
        MotorControl_MaxToMin(
            MOTOR_1,
            &parameters[PARAM_X1_P15V_I_MAX_MIN],
            &parameters[PARAM_X1_N15V_I_MAX_MIN]
        ) / 1000.0f;
    HAL_Delay(200);

    parameters[PARAM_X2_TIME_MIN_MAX] =
        MotorControl_MinToMax(
            MOTOR_2,
            &parameters[PARAM_X2_P15V_I_MIN_MAX],
            &parameters[PARAM_X2_N15V_I_MIN_MAX]
        ) / 1000.0f;
    HAL_Delay(200);

    parameters[PARAM_X2_TIME_MAX_MIN] =
        MotorControl_MaxToMin(
            MOTOR_2,
            &parameters[PARAM_X2_P15V_I_MAX_MIN],
            &parameters[PARAM_X2_N15V_I_MAX_MIN]
        ) / 1000.0f;
    HAL_Delay(200);

    /* -------- Smoothness tests -------- */
    parameters[PARAM_X1_STEP_MIN_MAX] =
        MotorControl_GetSmoothness_MinToMax(MOTOR_1);
    HAL_Delay(200);

    parameters[PARAM_X1_STEP_MAX_MIN] =
        MotorControl_GetSmoothness_MaxToMin(MOTOR_1);
    HAL_Delay(200);

    parameters[PARAM_X2_STEP_MIN_MAX] =
        MotorControl_GetSmoothness_MinToMax(MOTOR_2);
    HAL_Delay(200);

    parameters[PARAM_X2_STEP_MAX_MIN] =
        MotorControl_GetSmoothness_MaxToMin(MOTOR_2);
    HAL_Delay(200);

    /* -------- Send DATA packet -------- */
    printf("DATA,");
    for (int i = 0; i < 24; i++) {
        printf("%ld", (int32_t)(parameters[i] * 1000.0f));
        if (i < 23) printf(",");
    }
    printf("\n");
    fflush(stdout);

    g_test_is_running = 1;
}


/* ===== Corner positioning ===== */
/* Maps from: g_test_is_running == 2..5 */

void Test_GotoCorner(uint8_t corner_id)
{
    MotorCalibration_t *cal1 = MotorControl_GetCalibration(MOTOR_1);
    MotorCalibration_t *cal2 = MotorControl_GetCalibration(MOTOR_2);

    switch (corner_id) {

        case 1: /* MIN / MIN */
            printf("Corner 1: MIN / MIN\r\n");
            MotorControl_MoveToADC(MOTOR_1, cal1->adc_min, 10);
            MotorControl_MoveToADC(MOTOR_2, cal2->adc_min, 10);
            break;

        case 2: /* MIN / MAX */
            printf("Corner 2: MIN / MAX\r\n");
            MotorControl_MoveToADC(MOTOR_1, cal1->adc_min, 10);
            MotorControl_MoveToADC(MOTOR_2, cal2->adc_max, 10);
            break;

        case 3: /* MAX / MIN */
            printf("Corner 3: MAX / MIN\r\n");
            MotorControl_MoveToADC(MOTOR_1, cal1->adc_max, 10);
            MotorControl_MoveToADC(MOTOR_2, cal2->adc_min, 10);
            break;

        case 4: /* MAX / MAX */
            printf("Corner 4: MAX / MAX\r\n");
            MotorControl_MoveToADC(MOTOR_1, cal1->adc_max, 10);
            MotorControl_MoveToADC(MOTOR_2, cal2->adc_max, 10);
            break;

        default:
            printf("Invalid corner\r\n");
            break;
    }

    fflush(stdout);
    g_test_is_running = 1;
}

void Stop_test(void)
{
	Motor_StopAll();

}
