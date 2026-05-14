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
#define MOVE_TOLERANCE_ADC  50
#define REDUCE_EMF_MS       1000

/* ================= PID PARAMETERS ================= */

static float Kp = 2.0f;    //1.2f
static float Ki = 12.0f;   //5.0f
static float Kd = 0.3f;
#define BANDPWM 2200

/* ================= PID STATE ================= */

/* ================= PID STATE ================= */
typedef struct {
    float    integral;
    float    previousADC;
    float    d_filtered;     // NEW: filtered derivative
} PID_State_t;

#define PID_DT_S   0.001f
/* ================= TASK STATE (shared with ISR) ================= */
typedef struct {
    volatile uint8_t  active;
    volatile uint16_t targetADC;
    volatile uint16_t tolerance;
    volatile uint8_t  done_flag;
    volatile uint32_t start_tick;
    PID_State_t       pid;
} MotorTask_t;

static MotorTask_t mtask[2];
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
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&Pot1_2, 2);
    HAL_Delay(10);
}

/* ============== Fixed-rate PID (called from TIM6 ISR @ 1 kHz) ============== */
static float PID_Run_Fixed(PID_State_t *pid, float error, float currentADC)
{
    /* P */
    float P = Kp * error;

    /* I with conditional integration (anti-windup) */
    float pre_out = P + Ki * pid->integral;
    uint8_t sat_hi = (pre_out >  PWM_MAX_DUTY) && (error > 0);
    uint8_t sat_lo = (pre_out < -PWM_MAX_DUTY) && (error < 0);
    if (!sat_hi && !sat_lo && Ki > 0.0f) {
        pid->integral += error * PID_DT_S;
    }
    const float I_LIMIT = 2500.0f ;
    if (pid->integral >  I_LIMIT) pid->integral =  I_LIMIT;
    if (pid->integral < -I_LIMIT) pid->integral = -I_LIMIT;
    float I = Ki * pid->integral;

    /* D on measurement, low-pass filtered */
    float d_adc = (currentADC - pid->previousADC) / PID_DT_S;
    const float alpha = 0.15f;
    pid->d_filtered = alpha * d_adc + (1.0f - alpha) * pid->d_filtered;
    float D = -Kd * pid->d_filtered;

    pid->previousADC = currentADC;
    return P + I + D;
}
/* ================= ISR — runs at 1 kHz from TIM6 ================= */
void MotorControl_PID_ISR(void)
{
    for (int m = 0; m < 2; m++) {
        if (!mtask[m].active) continue;

        uint16_t curADC = Pot1_2[m];
        int32_t  error  = (int32_t)mtask[m].targetADC - (int32_t)curADC;

        /* Target reached? */
        if (labs(error) <= mtask[m].tolerance) {
            Motor_Stop((Motor_ID_t)m);
            mtask[m].pid.integral = 0.0f;
            mtask[m].active    = 0;
            mtask[m].done_flag = 1;
            continue;
        }

        float cmd = PID_Run_Fixed(&mtask[m].pid, (float)error, (float)curADC);
        Motor_Set((Motor_ID_t)m, cmd);
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
//    	pid_isr_count++;
        MotorControl_PID_ISR();
    }
}

/* ================= NON-BLOCKING API ================= */
void MotorControl_StartMove(Motor_ID_t motor, uint16_t targetADC, uint16_t tolerance)
{
    /* Disable interrupts briefly while we set up shared state */
    __disable_irq();
    mtask[motor].pid.integral    = 0.0f;
    mtask[motor].pid.previousADC = (float)Pot1_2[motor];
    mtask[motor].pid.d_filtered  = 0.0f;
    mtask[motor].targetADC       = targetADC;
    mtask[motor].tolerance       = tolerance;
    mtask[motor].done_flag       = 0;
    mtask[motor].start_tick      = HAL_GetTick();
    mtask[motor].active          = 1;        // ISR picks this up next tick
    __enable_irq();

    /* Reset peak current trackers */
    if (motor == MOTOR_1) {
        Max_PCurrent_M1 = Max_NCurrent_M1 = Max_24VCurrent_M1 = 0.0f;
    } else {
        Max_PCurrent_M2 = Max_NCurrent_M2 = Max_24VCurrent_M2 = 0.0f;
    }
}
uint8_t MotorControl_IsBusy(Motor_ID_t motor) { return mtask[motor].active; }

void MotorControl_Abort(Motor_ID_t motor)
{
    mtask[motor].active = 0;
    Motor_Stop(motor);
}

/* ================= I2C SERVICE (called from main / wait loops) ================= */
void MotorControl_ServiceI2C(Motor_ID_t motor)
{
    static uint32_t last_tick[2] = {0, 0};
    if (HAL_GetTick() - last_tick[motor] < 20) return;   // gate to 50 Hz
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

void MotorControl_MoveToADC(Motor_ID_t motor, uint16_t targetADC, uint16_t tolerance)
{
    MotorControl_StartMove(motor, targetADC, tolerance);

    while (mtask[motor].active) {
        MotorControl_ServiceI2C(motor);

        if (HAL_GetTick() - mtask[motor].start_tick > MOVE_TIMEOUT_MS) {
            MotorControl_Abort(motor);
            return;
        }
    }
}
void MotorControl_MoveAndMeasureSmoothness(
        Motor_ID_t motor, uint16_t targetADC,
        uint32_t expected_duration_ms, uint16_t tolerance,
        int32_t *max_error_out)
{
    if (max_error_out == NULL) return;
    *max_error_out = 0;

    uint16_t startADC = Pot1_2[motor];
    uint32_t startTick = HAL_GetTick();

    MotorControl_StartMove(motor, targetADC, tolerance);

    while (mtask[motor].active) {
        MotorControl_ServiceI2C(motor);

        uint32_t elapsed = HAL_GetTick() - startTick;
        if (expected_duration_ms > 0 && elapsed <= expected_duration_ms) {
            int32_t ideal = (int32_t)startADC +
                            ((int32_t)(targetADC - startADC) * (int32_t)elapsed) /
                            (int32_t)expected_duration_ms;
            int32_t smooth_err = (int32_t)Pot1_2[motor] - ideal;
            if (labs(smooth_err) > labs(*max_error_out))
                *max_error_out = smooth_err;
        }

        if (elapsed > MOVE_TIMEOUT_MS) {
            MotorControl_Abort(motor);
            return;
        }
    }
}

/* ================= CALIBRATION ================= */
/* Maps from:
 * FindMinPositionVoltageMotorX
 * FindMaxPositionVoltageMotorX
 */
float MotorControl_FindMax(Motor_ID_t motor)
{
	 MotorControl_Abort(motor);
    uint16_t lastADC = Pot1_2[motor];
    uint32_t start = HAL_GetTick();
    float voltage = -1.0f;

    Motor_Set(motor, BANDPWM);   //1700

    while (1)
    {
        HAL_Delay(100);
        uint16_t cur = Pot1_2[motor];

        if (abs((int32_t)cur - (int32_t)lastADC) <= 20)
        {
//        	Motor_Stop(motor);

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

    motorCal[motor].adc_max = avgADC - 100;

    /* Read Voltage */
    if (I2C_Init_INA226(&hi2c1, motor == MOTOR_1 ? Pot1 : Pot2, 0x4FE6, 0x1000) != HAL_OK)
        printf("Failed to initialize Pot1/2 INA226\r\n");
    else{
    	printf("Pot1/2 INA226 initialized\r\n");
    }
	if (I2C_WaitForConversionPolling(&hi2c1, motor == MOTOR_1 ? Pot1 : Pot2, 1024, 9.344f) != HAL_OK) {
	    printf("Polling timeout or I2C error!\r\n");
	}
    I2C_ReadVoltage(&hi2c1,
                    motor == MOTOR_1 ? Pot1 : Pot2,
                    &voltage);
    Motor_Stop(motor);

    return voltage;
}
float MotorControl_FindMin(Motor_ID_t motor)
{
	 MotorControl_Abort(motor);
    uint16_t lastADC = Pot1_2[motor];
    uint32_t start = HAL_GetTick();
    float voltage = -1.0f;

    Motor_Set(motor, -BANDPWM);
    while (1)
    {
        HAL_Delay(100);
        uint16_t cur = Pot1_2[motor];

        if (abs((int32_t)cur - (int32_t)lastADC) <= 20)
        {
//        	Motor_Stop(motor);

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

    motorCal[motor].adc_min = avgADC + 100;

    /* Read Voltage */
    if (I2C_Init_INA226(&hi2c1, motor == MOTOR_1 ? Pot1 : Pot2, 0x4FE6, 0x1000) != HAL_OK)
        printf("Failed to initialize Pot1/2 INA226\r\n");
    else{
    	printf("Pot1/2 INA226 initialized\r\n");
    }
	if (I2C_WaitForConversionPolling(&hi2c1, motor == MOTOR_1 ? Pot1 : Pot2, 1024, 9.344f) != HAL_OK) {
	    printf("Polling timeout or I2C error!\r\n");
	}
    I2C_ReadVoltage(&hi2c1,
                    motor == MOTOR_1 ? Pot1 : Pot2,
                    &voltage);
    Motor_Stop(motor);

    return voltage;
}

void MotorControl_UpdateHome(void)
{
    for (int i = 0; i < 2; i++) {
        motorCal[i].adc_home =
            (motorCal[i].adc_min + motorCal[i].adc_max) / 2;
    }
    MotorControl_MoveToADC(MOTOR_1, motorCal[MOTOR_1].adc_home, MOVE_TOLERANCE_ADC);
    MotorControl_MoveToADC(MOTOR_2, motorCal[MOTOR_2].adc_home, MOVE_TOLERANCE_ADC);

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
