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
#include "relay_driver.h"

/* ===== External application data ===== */

extern float parameters[];
extern volatile uint8_t g_test_is_running;

/* These are still global measurement outputs */
extern float Max_PCurrent_M1;
extern float Max_PCurrent_M2;

float parameters[NUM_PARAMETERS] = {0.0f};

float currentLog[2000];
uint16_t logIndex = 0;


/* ===== Init ===== */

void Test_Init(void)
{
    SineGenerator_Start(&hdac, &htim6);
    I2C_Scan(&hi2c1);
    HAL_Delay(1000);
    const uint16_t currentConfig = 0x4B7F;
    const float Maximum_Expected_Current = 0.7f;
    const float Shunt_Resistor_Value = 0.1f;
    const float Current_LSB = Maximum_Expected_Current / 32768.0f;
    const float base_cal = 0.00512f / (Current_LSB * Shunt_Resistor_Value);
    const uint16_t calibration_value = (uint16_t)(base_cal + 0.5f);


    // Init current-sensing INA226s
    if (I2C_Init_INA226(&hi2c1, T15VP,currentConfig, calibration_value) != HAL_OK)
        printf("Failed to initialize +15V INA226\r\n");
    else
        printf(" +15V INA226 initialized\r\n");

    HAL_Delay(100);

    if (I2C_Init_INA226(&hi2c1, T24V, currentConfig, calibration_value) != HAL_OK)
        printf("Failed to initialize 24V INA226\r\n");
    else
        printf(" 24V INA226 initialized\r\n");

    HAL_Delay(100);

    // Init voltage-only INA226s (Pot1 & Pot2)
    const uint16_t potVoltageConfig    = 0x4FE6;
    const uint16_t potVoltageCalReg    = 0x1000;  // Arbitrary valid calibration value

    if (I2C_Init_INA226(&hi2c1, Pot1, potVoltageConfig, potVoltageCalReg) != HAL_OK)
        printf("Failed to initialize Pot1 INA226\r\n");
    else
        printf("Pot1 INA226 initialized\r\n");

    HAL_Delay(100);

    if (I2C_Init_INA226(&hi2c1, Pot2, potVoltageConfig, potVoltageCalReg) != HAL_OK)
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
			&parameters[PARAM_X1_N15V_I_MIN_MAX],
            &parameters[PARAM_X1_24V_I_MIN_MAX]
        ) / 1000.0f;
    HAL_Delay(200);

    parameters[PARAM_X1_TIME_MAX_MIN] =
        MotorControl_MaxToMin(
            MOTOR_1,
            &parameters[PARAM_X1_P15V_I_MAX_MIN],
			&parameters[PARAM_X1_N15V_I_MAX_MIN],
            &parameters[PARAM_X1_24V_I_MAX_MIN]
        ) / 1000.0f;
    HAL_Delay(200);

    parameters[PARAM_X2_TIME_MIN_MAX] =
        MotorControl_MinToMax(
            MOTOR_2,
            &parameters[PARAM_X2_P15V_I_MIN_MAX],
			&parameters[PARAM_X2_N15V_I_MIN_MAX],
            &parameters[PARAM_X2_24V_I_MIN_MAX]
        ) / 1000.0f;
    HAL_Delay(200);

    parameters[PARAM_X2_TIME_MAX_MIN] =
        MotorControl_MaxToMin(
            MOTOR_2,
            &parameters[PARAM_X2_P15V_I_MAX_MIN],
			&parameters[PARAM_X2_N15V_I_MAX_MIN],
            &parameters[PARAM_X2_24V_I_MAX_MIN]
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
    MotorControl_UpdateHome();

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

    switch (corner_id) {

        case 1:
        	printf("Relay Config 1 On \r\n");
        	set_relay_combination(1);
            break;

        case 2: /* MIN / MAX */
        	printf("Relay Config 2 On \r\n");
        	set_relay_combination(2);
            break;

        case 3: /* MAX / MIN */
        	printf("Relay Config 3 On \r\n");
        	set_relay_combination(4);
            break;

        case 4: /* MAX / MAX */
        	printf("All Relay off \r\n");
        	set_relay_combination(0);
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

void Debug_RunMotorConstantPWM(void)
{
    float instCurrent;

    logIndex = 0;

    /* Start motor at constant PWM */
    Motor_Set(MOTOR_1, 130.0f);   // Example constant PWM

    for(int i = 0; i < 2000; i++)
    {
        if(I2C_ReadCurrent(&hi2c1, T15VP, &instCurrent) == HAL_OK)
        {
            currentLog[logIndex++] = instCurrent;
        }

        HAL_Delay(1);   // sample every 1 ms
    }

    /* Stop motor */
    Motor_Stop(MOTOR_1);

    /* Print logged data */
    for(uint16_t i = 0; i < logIndex; i++)
    {
        printf("%ld\n", (int32_t)(currentLog[i] * 1000));
    }
}

