#include "motor_control.h"
#include "motor_driver.h"
#include "i2ccomm.h"
#include "main.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>



/* ================= CONFIG ================= */

#define MOVE_TIMEOUT_MS     10000
#define SETTLE_DELAY_MS     100
#define LOOP_DELAY_MS       1
#define MOVE_TOLERANCE_ADC  75
#define REDUCE_EMF_MS       1000

/* ================= PID PARAMETERS ================= */

static float Kp = 4.0f;
static float Ki = 0.2f;
static float Kd = 0.000f;


/* ================= PID STATE ================= */

typedef struct {
    float integral;
    float previousADC;
    uint32_t prev_tick;
} PID_State_t;

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
    {530, 2230, 0},   // MOTOR_1 defaults
    {530, 2230, 0}    // MOTOR_2 defaults
};

static PID_State_t pidState[2];
uint16_t volatile Pot1_2[2];



/* ================= INTERNAL PID ================= */
/* Maps from:
 * PID_Control()
 */
static float PID_Run(PID_State_t *pid,
                     float error,
                     float currentADC,
                     uint8_t reset)
{
    uint32_t now = HAL_GetTick();

    /* ===== Reset PID state ===== */
    if (reset) {
        pid->integral     = 0.0f;
        pid->previousADC = currentADC;
        pid->prev_tick   = now;
        return 0.0f;
    }

    /* ===== Time delta (ms → s) ===== */
    float dt = (now - pid->prev_tick) * 0.001f;
    if (dt < 0.001f)
        dt = 0.001f;   // protects against tick jitter / same-tick calls

    /* ===== Proportional ===== */
    float P = Kp * error;

    /* ===== Integral with anti-windup ===== */
    pid->integral += error * dt;

    /* Integral clamp (CRITICAL for motors) */
    const float I_LIMIT = (Ki > 0.0f) ? (200.0f / Ki) : 0.0f; // keeps output within ±200
    if (pid->integral >  I_LIMIT) pid->integral =  I_LIMIT;
    if (pid->integral < -I_LIMIT) pid->integral = -I_LIMIT;

    float I = Ki * pid->integral;

    /* ===== Derivative (on measurement, noise-safe) ===== */
    float d_adc = (currentADC - pid->previousADC) / dt;
    float D = -Kd * d_adc;

    /* ===== Update state ===== */
    pid->previousADC = currentADC;
    pid->prev_tick   = now;

    return P + I + D;
}


/* ================= INIT ================= */

void MotorControl_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&Pot1_2, 2);
}

/* ================= CALIBRATION ================= */
/* Maps from:
 * FindMinPositionVoltageMotorX
 * FindMaxPositionVoltageMotorX
 */
float MotorControl_FindMin(Motor_ID_t motor)
{
    uint16_t lastADC = Pot1_2[motor];
    uint32_t start = HAL_GetTick();
    float voltage = -1.0f;

    Motor_Set(motor, -120.0f);

    while (1)
    {
        HAL_Delay(100);
        uint16_t cur = Pot1_2[motor];

        if (abs((int32_t)cur - (int32_t)lastADC) <= 20)
        {
            Motor_Stop(motor);
            break;
        }

        lastADC = cur;

        if (HAL_GetTick() - start > MOVE_TIMEOUT_MS)
        {
            Motor_Stop(motor);
            break;
        }
    }

    /* ===== Average 500 ADC Samples ===== */
    uint32_t sum = 0;

    for (int i = 0; i < 500; i++)
    {
        sum += Pot1_2[motor];
        HAL_Delay(2);    // 2ms → 1 second averaging window
    }

    uint16_t avgADC = (uint16_t)(sum / 500);

    motorCal[motor].adc_min = avgADC;

    /* Read Voltage */
    I2C_ReadVoltage(&hi2c1,
                    motor == MOTOR_1 ? Pot1 : Pot2,
                    &voltage);

    return voltage;
}


float MotorControl_FindMax(Motor_ID_t motor)
{
    uint16_t lastADC = Pot1_2[motor];
    uint32_t start = HAL_GetTick();
    float voltage = -1.0f;

    Motor_Set(motor, 120.0f);

    while (1)
    {
        HAL_Delay(100);
        uint16_t cur = Pot1_2[motor];

        if (abs((int32_t)cur - (int32_t)lastADC) <= 20)
        {
            Motor_Stop(motor);
            break;
        }

        lastADC = cur;

        if (HAL_GetTick() - start > MOVE_TIMEOUT_MS)
        {
            Motor_Stop(motor);
            break;
        }
    }

    /* ===== Average 500 ADC Samples ===== */
    uint32_t sum = 0;

    for (int i = 0; i < 500; i++)
    {
        sum += Pot1_2[motor];
        HAL_Delay(2);
    }

    uint16_t avgADC = (uint16_t)(sum / 500);

    motorCal[motor].adc_max = avgADC;

    /* Read Voltage */
    I2C_ReadVoltage(&hi2c1,
                    motor == MOTOR_1 ? Pot1 : Pot2,
                    &voltage);

    return voltage;
}

void MotorControl_UpdateHome(void)
{
    for (int i = 0; i < 2; i++) {
        motorCal[i].adc_home =
            (motorCal[i].adc_min + motorCal[i].adc_max) / 2;
    }
}

/* ================= MOTION ================= */
/* Maps from:
 * moveMotorXToADCValue
 */

void MotorControl_MoveToADC(Motor_ID_t motor,
                            uint16_t targetADC,
                            uint16_t tolerance)
{
    PID_State_t *pid = &pidState[motor];
    uint32_t start = HAL_GetTick();
    float I24 = 0.0f;
    float I15P = 0.0f;
    float I15N = 0.0f;
    float Power24V;
    float Power15V;
    float Power15VN;


    if (motor == MOTOR_1) {
        Max_PCurrent_M1 = 0.0f;
        Max_NCurrent_M1 = 0.0f;
        Max_24VCurrent_M1 = 0.0f;

    } else {
        Max_PCurrent_M2 = 0.0f;
        Max_NCurrent_M2 = 0.0f;
        Max_24VCurrent_M2 = 0.0f;
    }

    PID_Run(pid, 0.0f, (float)Pot1_2[motor], 1);

    while (1) {
        uint16_t curADC = Pot1_2[motor];
        int32_t error = (int32_t)targetADC - (int32_t)curADC;

        if (I2C_ReadCurrent(&hi2c1, T24V, &I24) == HAL_OK) {
            float absI = fabsf(I24);
            if (motor == MOTOR_1) {
                if (absI > Max_24VCurrent_M1) Max_24VCurrent_M1 = absI;
            } else {
                if (absI > Max_24VCurrent_M2) Max_24VCurrent_M2 = absI;
            }
        }
        Power24V = (24.0)*I24;

        if (I2C_ReadCurrent(&hi2c1, T15VP, &I15P) == HAL_OK) {
            float absI = fabsf(I15P);
            if (motor == MOTOR_1) {
                if (absI > Max_PCurrent_M1) Max_PCurrent_M1 = absI;
            } else {
                if (absI > Max_PCurrent_M2) Max_PCurrent_M2 = absI;
            }
        }
        Power15V = (15.0)*I15P ;

        Power15VN = Power24V - Power15V - ConstantPower;
        I15N = Power15VN/15 ;
        float absI = fabsf(I15N);
        if (motor == MOTOR_1)
        {
                        if (absI > Max_NCurrent_M1) Max_NCurrent_M1 = absI;
                    } else {
                        if (absI > Max_NCurrent_M2) Max_NCurrent_M2 = absI;
                    }
        if (labs(error) <= tolerance) {
            Motor_Stop(motor);
            return;
        }

        float cmd = PID_Run(pid, (float)error, (float)curADC, 0);

        Motor_Set(motor, cmd);

        if ((HAL_GetTick() - start) > MOVE_TIMEOUT_MS) {
            Motor_Stop(motor);
            return;
        }
    }
}


/* ================= SMOOTHNESS ================= */
/* Maps from:
 * moveMotorXToADCValue_And_Measure_Smoothness
 */

void MotorControl_MoveAndMeasureSmoothness(
        Motor_ID_t motor,
        uint16_t targetADC,
        uint32_t expected_duration_ms,
        uint16_t tolerance,
        int32_t *max_error_out)
{
    uint16_t startADC = Pot1_2[motor];
    uint32_t startTick = HAL_GetTick();
    PID_State_t *pid = &pidState[motor];

    if (max_error_out == NULL)
        return;

    *max_error_out = 0;

    /* Reset PID */
    PID_Run(pid, 0.0f, (float)startADC, 1);

    while (1) {

        uint16_t curADC = Pot1_2[motor];
        int32_t error = (int32_t)targetADC - (int32_t)curADC;

        /* Target reached */
        if (labs(error) <= tolerance) {
            Motor_Stop(motor);
            return;
        }

        /* ===== Smoothness calculation ===== */
        uint32_t elapsed = HAL_GetTick() - startTick;

        if (expected_duration_ms > 0 && elapsed <= expected_duration_ms) {

        	int32_t ideal =
        	    (int32_t)startADC +
        	    (int32_t)((float)(targetADC - startADC) *
        	              ((float)elapsed / (float)expected_duration_ms));

            int32_t smooth_err = (int32_t)curADC - ideal;

            if (labs(smooth_err) > labs(*max_error_out))
                *max_error_out = smooth_err;
        }

        /* ===== PID control ===== */
        float cmd = PID_Run(pid,
                            (float)error,
                            (float)curADC,
                            0);
        Motor_Set(motor, cmd);

        /* Timeout protection */
        if ((HAL_GetTick() - startTick) > MOVE_TIMEOUT_MS) {
            Motor_Stop(motor);
            return;
        }


    }
}


/* ================= TEST HELPERS ================= */
/* Maps from:
 * ADC_MIN_TO_ADC_MAX_MX
 * ADC_MAX_TO_ADC_MIN_MX
 */

uint32_t MotorControl_MinToMax(Motor_ID_t motor, float *P15, float *N15,float *V24)
{
    /* Go to MIN first (no timing) */
    MotorControl_MoveToADC(motor, motorCal[motor].adc_min, MOVE_TOLERANCE_ADC);


    HAL_Delay(REDUCE_EMF_MS);
    uint32_t t0 = HAL_GetTick();

    /* Timed move MIN → MAX */
    MotorControl_MoveToADC(motor, motorCal[motor].adc_max, MOVE_TOLERANCE_ADC);


    uint32_t t1 = HAL_GetTick();

    /* Return peak currents */
    if (P15 != NULL && N15 != NULL && V24 !=NULL) {
        if (motor == MOTOR_1) {
            *P15 = Max_PCurrent_M1;
            *N15 = Max_NCurrent_M1;
            *V24 = Max_24VCurrent_M1;
        } else {
            *P15 = Max_PCurrent_M2;
            *N15 = Max_NCurrent_M2;
            *V24 = Max_24VCurrent_M2;
        }
    }

    /* Return to HOME */
    MotorControl_MoveToADC(motor, motorCal[motor].adc_home, MOVE_TOLERANCE_ADC);

    return (t1 - t0);
}

uint32_t MotorControl_MaxToMin(Motor_ID_t motor, float *P15, float *N15,float *V24)
{
    /* Go to MAX first (no timing) */
    MotorControl_MoveToADC(motor, motorCal[motor].adc_max, MOVE_TOLERANCE_ADC);
    HAL_Delay(REDUCE_EMF_MS);

    uint32_t t0 = HAL_GetTick();

    /* Timed move MAX → MIN */
    MotorControl_MoveToADC(motor, motorCal[motor].adc_min, MOVE_TOLERANCE_ADC);

    uint32_t t1 = HAL_GetTick();

    /* Return peak currents */
    if (P15 != NULL && N15 != NULL && V24 !=NULL) {
        if (motor == MOTOR_1) {
            *P15 = Max_PCurrent_M1;
            *N15 = Max_NCurrent_M1;
            *V24 = Max_24VCurrent_M1;
        } else {
            *P15 = Max_PCurrent_M2;
            *N15 = Max_NCurrent_M2;
            *V24 = Max_24VCurrent_M2;
        }
    }

    /* Return to HOME */
    MotorControl_MoveToADC(motor, motorCal[motor].adc_home, MOVE_TOLERANCE_ADC);

    return (t1 - t0);
}

float MotorControl_GetSmoothness_MinToMax(Motor_ID_t motor)
{
    float pP;
    float pN;
    float pV;

    uint32_t duration =
        MotorControl_MinToMax(motor, &pP, &pN, &pV);

    HAL_Delay(SETTLE_DELAY_MS);

    int32_t max_err = 0;
    MotorControl_MoveAndMeasureSmoothness(
        motor,
        motorCal[motor].adc_max,
        duration,
        MOVE_TOLERANCE_ADC,
        &max_err
    );

    float dist =
        (float)(motorCal[motor].adc_max - motorCal[motor].adc_min);

    return (dist > 0.0f) ? ((float)max_err / dist) : 0.0f;
}


float MotorControl_GetSmoothness_MaxToMin(Motor_ID_t motor)
{
    float pP;
    float pN;
    float pV;

    uint32_t duration =
        MotorControl_MaxToMin(motor, &pP, &pN,&pV);

    HAL_Delay(SETTLE_DELAY_MS);

    int32_t max_err = 0;
    MotorControl_MoveAndMeasureSmoothness(
        motor,
        motorCal[motor].adc_min,
        duration,
        MOVE_TOLERANCE_ADC,
        &max_err
    );

    float dist =
        (float)(motorCal[motor].adc_max - motorCal[motor].adc_min);

    return (dist > 0.0f) ? ((float)max_err / dist) : 0.0f;
}



/* ================= ACCESS ================= */

MotorCalibration_t* MotorControl_GetCalibration(Motor_ID_t motor)
{
    return &motorCal[motor];
}
