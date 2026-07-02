#include "motor_control.h"
#include "motor_driver.h"
#include "i2ccomm.h"
#include "main.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include "DAC_Generator.h"
#include <stdbool.h>


/* ================= CONFIG ================= */

#define MOVE_TIMEOUT_MS     10000
#define SETTLE_DELAY_MS     100
#define LOOP_DELAY_MS       1
#define MOVE_TOLERANCE_ADC  50
#define REDUCE_EMF_MS       1000
#define ADC_MOTOR1_INDEX   1
#define ADC_MOTOR2_INDEX   0




/* ================= Global Current&Voltage Values ================= */
float Max_PCurrent_M1 = 0.0f;
float Max_NCurrent_M1 = 0.0f;
float Max_24VCurrent_M1 = 0.0f;
float Voltage_M1      = 0.0f;;
float Max_PCurrent_M2 = 0.0f;
float Max_NCurrent_M2 = 0.0f;
float Max_24VCurrent_M2 = 0.0f;
float Voltage_M2      = 0.0f;
float ConstantPower = 1.632f; //1.416
/* ================= GLOBAL STATE ================= */

/* Maps from:
 * ADC_POS_MIN_M1 / ADC_POS_MAX_M1 / ADC_POS_HOME_M1
 * ADC_POS_MIN_M2 / ADC_POS_MAX_M2 / ADC_POS_HOME_M2
 */
static MotorCalibration_t motorCal[2] = {
    {370, 3250, 0},   // MOTOR_1 defaults at 370 it Just hits the switch .
    {370, 3250, 0}    // MOTOR_2 defaults
};

volatile uint32_t pid_isr_count = 0;

uint16_t volatile Pot1_2[2];
/* ================= INIT ================= */

void MotorControl_Init(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // Bit Low for Manual Configration
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	DACGenerator_Start(&hdac, DAC_CHANNEL_1, 2048);
	DACGenerator_Start(&hdac, DAC_CHANNEL_2, 2048);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&Pot1_2, 2);
    HAL_Delay(10);
}

static inline uint16_t Motor_GetADC(Motor_ID_t motor)
{
    if (motor == MOTOR_1)
        return Pot1_2[ADC_MOTOR1_INDEX];
    else
        return Pot1_2[ADC_MOTOR2_INDEX];
}

void MotorControl_Abort(Motor_ID_t motor)
{
    Motor_Stop(motor);
}

/* ================= I2C SERVICE (called from main / wait loops) ================= */
void MotorControl_ServiceI2C(Motor_ID_t motor)
{
    static uint32_t last_tick[2] = {0, 0};
    if (HAL_GetTick() - last_tick[motor] < 5) return;   // gate to 50 Hz
    last_tick[motor] = HAL_GetTick();

    float I24 = 0.0f, I15P = 0.0f;

    if (I2C_ReadCurrent(&hi2c1, T24V, &I24) == HAL_OK) {
        float a = fabsf(I24);
        if (motor == MOTOR_1) { if (a > Max_24VCurrent_M1) Max_24VCurrent_M1 = a; }
        else                  { if (a > Max_24VCurrent_M2) Max_24VCurrent_M2 = a; }
    }
    if (I2C_ReadCurrent(&hi2c1, T15VP, &I15P) == HAL_OK) {
        float a = fabsf(I15P);
        if (motor == MOTOR_1) { if (a > Max_PCurrent_M1) Max_PCurrent_M1 = a; }
        else                  { if (a > Max_PCurrent_M2) Max_PCurrent_M2 = a; }
    }

//    float P15N = 24.0f*I24 - 15.0f*I15P - ConstantPower;
//    float a    = fabsf(P15N / 15.0f);
    float a = 0.0f;
    if (motor == MOTOR_1) { if (a > Max_NCurrent_M1) Max_NCurrent_M1 = a; }
    else                  { if (a > Max_NCurrent_M2) Max_NCurrent_M2 = a; }
}

void MotorControl_MoveToADC(Motor_ID_t motor, uint16_t targetPOS , uint16_t targetADC)
{
;

    uint16_t currADC;
    if (motor == MOTOR_1) {
        Max_PCurrent_M1 = Max_NCurrent_M1 = Max_24VCurrent_M1 = 0.0f;
    } else {
        Max_PCurrent_M2 = Max_NCurrent_M2 = Max_24VCurrent_M2 = 0.0f;
    }

    uint32_t start = HAL_GetTick();
    Motor_Set(motor, targetPOS);
    uint8_t stable_count = 0;

    while(1)
    {

        MotorControl_ServiceI2C(motor);

        currADC = Motor_GetADC(motor);

        if (abs((int32_t)currADC - (int32_t)targetADC)
                <= MOVE_TOLERANCE_ADC)
        {
            stable_count++;

            if(stable_count >= 3)
                return;
        }
        else
        {
            stable_count = 0;
        }

        if(HAL_GetTick() - start > MOVE_TIMEOUT_MS)
        {

            return;
        }

        HAL_Delay(1);
    }
}
void MotorControl_MoveAndMeasureSmoothness(
        Motor_ID_t motor,
        uint16_t targetPOS,
		uint16_t targetADC,
        uint32_t expected_duration_ms,
        int32_t *max_error_out)
{


    if (max_error_out == NULL)
        return;

    *max_error_out = 0;

    uint16_t startADC = Motor_GetADC(motor);
    uint32_t startTick = HAL_GetTick();

    Motor_Set(motor, targetPOS);
    uint8_t stable_count = 0;

    while (1)
    {

    	uint16_t currADC = Motor_GetADC(motor);

    	uint32_t elapsed = HAL_GetTick() - startTick;

    	if (expected_duration_ms > 0 &&
    	    elapsed <= expected_duration_ms)
    	{
    	    int32_t ideal =
    	        (int32_t)startADC +
    	        ((int32_t)(targetADC - startADC) *
    	         (int32_t)elapsed) /
    	        (int32_t)expected_duration_ms;

    	    int32_t smooth_err =
    	        (int32_t)currADC - ideal;

    	    if (labs(smooth_err) > labs(*max_error_out))
    	    {
    	        *max_error_out = smooth_err;
    	    }
    	}

    	if (abs((int32_t)currADC - (int32_t)targetADC)
    	        <= MOVE_TOLERANCE_ADC)
    	{
    	    if (++stable_count >= 3)
    	        return;
    	}
    	else
    	{
    	    stable_count = 0;
    	}
    	if ((HAL_GetTick() - startTick) > MOVE_TIMEOUT_MS)
    	{
    	    return;
    	}
    	HAL_Delay(1);
}
/* ================= CALIBRATION ================= */
/* Maps from:
 * FindMinPositionVoltageMotorX
 * FindMaxPositionVoltageMotorX
 */
float MotorControl_FindMax(Motor_ID_t motor)
{


    uint16_t lastADC = Motor_GetADC(motor);
    uint32_t start = HAL_GetTick();
    float voltage = -1.0f;

    uint8_t stable_count = 0;

    Motor_Set(motor, MAX_POS);

    while (1)
    {
        HAL_Delay(100);

        uint16_t cur = Motor_GetADC(motor);

        if (abs((int32_t)cur - (int32_t)lastADC) <= 20)
        {
            stable_count++;

            if (stable_count >= 3)
                break;
        }
        else
        {
            stable_count = 0;
        }

        lastADC = cur;

        if ((HAL_GetTick() - start) > MOVE_TIMEOUT_MS)
        {
            break;
        }
    }

    /* Average settled ADC value */
    uint32_t sum = 0;

    for (uint16_t i = 0; i < 500; i++)
    {
        sum += Motor_GetADC(motor);
        HAL_Delay(2);
    }

    motorCal[motor].adc_max = (uint16_t)(sum / 500);

    voltage =
        ((float)motorCal[motor].adc_max * 10.0f) /
        4095.0f;



    return voltage;
}
float MotorControl_FindMin(Motor_ID_t motor)
{


    uint16_t lastADC = Motor_GetADC(motor);
    uint32_t start = HAL_GetTick();
    float voltage = -1.0f;

    uint8_t stable_count = 0;

    Motor_Set(motor, MIN_POS);

    while (1)
    {
        HAL_Delay(100);

        uint16_t cur = Motor_GetADC(motor);

        if (abs((int32_t)cur - (int32_t)lastADC) <= 20)
        {
            stable_count++;

            if (stable_count >= 3)
                break;
        }
        else
        {
            stable_count = 0;
        }

        lastADC = cur;

        if ((HAL_GetTick() - start) > MOVE_TIMEOUT_MS)
        {
            break;
        }
    }

    /* Average settled ADC value */
    uint32_t sum = 0;

    for (uint16_t i = 0; i < 500; i++)
    {
        sum += Motor_GetADC(motor);
        HAL_Delay(2);
    }

    motorCal[motor].adc_min = (uint16_t)(sum / 500);

    voltage =
        ((float)motorCal[motor].adc_min * 10.0f) /
        4095.0f;



    return voltage;
}

void MotorControl_UpdateHome(void)
{
	motorCal[MOTOR_1].adc_home =
	    (motorCal[MOTOR_1].adc_min + motorCal[MOTOR_1].adc_max) / 2;

	motorCal[MOTOR_2].adc_home =
	    (motorCal[MOTOR_2].adc_min + motorCal[MOTOR_2].adc_max) / 2;

	MotorControl_MoveToADC(
	    MOTOR_1,
	    MID_POS,
	    motorCal[MOTOR_1].adc_home
	);

	MotorControl_MoveToADC(
	    MOTOR_2,
	    MID_POS,
	    motorCal[MOTOR_2].adc_home
	);
}

/* ================= TEST HELPERS ================= */
/* Maps from:
 * ADC_MIN_TO_ADC_MAX_MX
 * ADC_MAX_TO_ADC_MIN_MX
 */

uint32_t MotorControl_MinToMax(
        Motor_ID_t motor,
        float *P15,
        float *N15,
        float *V24)
{
    /* Go to MIN first */
	uint32_t t0 = HAL_GetTick();
    MotorControl_MoveToADC(motor, MIN_POS,motorCal[motor].adc_min);



    /* Timed move MIN -> MAX */
    MotorControl_MoveToADC(motor, MAX_POS, motorCal[motor].adc_max);

    uint32_t t1 = HAL_GetTick();

    /* Return peak currents */
    if (P15 && N15 && V24)
    {
        if (motor == MOTOR_1)
        {
            *P15 = Max_PCurrent_M1;
            *N15 = Max_NCurrent_M1;
            *V24 = Max_24VCurrent_M1;
        }
        else
        {
            *P15 = Max_PCurrent_M2;
            *N15 = Max_NCurrent_M2;
            *V24 = Max_24VCurrent_M2;
        }
    }

    /* Return to HOME */
    MotorControl_MoveToADC(motor, MID_POS,motorCal[motor].adc_home);

    return (t1 - t0);
}
uint32_t MotorControl_MaxToMin(
        Motor_ID_t motor,
        float *P15,
        float *N15,
        float *V24)
{
    /* Go to MAX first */
	uint32_t t0 = HAL_GetTick();
    MotorControl_MoveToADC(motor, MAX_POS, motorCal[motor].adc_max);



    /* Timed move MAX -> MIN */
    MotorControl_MoveToADC(motor, MIN_POS, motorCal[motor].adc_min);

    uint32_t t1 = HAL_GetTick();

    /* Return peak currents */
    if (P15 && N15 && V24)
    {
        if (motor == MOTOR_1)
        {
            *P15 = Max_PCurrent_M1;
            *N15 = Max_NCurrent_M1;
            *V24 = Max_24VCurrent_M1;
        }
        else
        {
            *P15 = Max_PCurrent_M2;
            *N15 = Max_NCurrent_M2;
            *V24 = Max_24VCurrent_M2;
        }
    }

    /* Return to HOME */
    MotorControl_MoveToADC(motor, MID_POS,motorCal[motor].adc_home);

    return (t1 - t0);
}

float MotorControl_GetSmoothness_MinToMax(
        Motor_ID_t motor)
{
    float pP;
    float pN;
    float pV;

    uint32_t duration =
        MotorControl_MinToMax(
            motor,
            &pP,
            &pN,
            &pV);

    HAL_Delay(SETTLE_DELAY_MS);

    int32_t max_err = 0;
    MotorControl_MoveToADC(motor, MIN_POS,motorCal[motor].adc_min);

    MotorControl_MoveAndMeasureSmoothness(
        motor,
        MAX_POS,
		motorCal[motor].adc_min,
        duration,
        &max_err);

    float dist =
        (float)(motorCal[motor].adc_max -
                motorCal[motor].adc_min);

    return (dist > 0.0f)
        ? ((float)labs(max_err) / dist)
        : 0.0f;
}

float MotorControl_GetSmoothness_MaxToMin(
        Motor_ID_t motor)
{
    float pP;
    float pN;
    float pV;

    uint32_t duration =
        MotorControl_MaxToMin(
            motor,
            &pP,
            &pN,
            &pV);

    HAL_Delay(SETTLE_DELAY_MS);


    MotorControl_MoveToADC(motor, MAX_POS,motorCal[motor].adc_max);

    int32_t max_err = 0;

    MotorControl_MoveAndMeasureSmoothness(
        motor,
        MIN_POS,
		motorCal[motor].adc_min,
        duration,
        &max_err);

    float dist =
        (float)(motorCal[motor].adc_max -
                motorCal[motor].adc_min);

    return (dist > 0.0f)
        ? ((float)labs(max_err) / dist)
        : 0.0f;
}
