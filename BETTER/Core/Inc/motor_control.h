#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include "motor_driver.h"
#include "stm32f4xx_hal.h"


/* ================= Calibration ================= */

typedef struct {
    uint16_t adc_min;
    uint16_t adc_max;
    uint16_t adc_home;
} MotorCalibration_t;

/* ================= Init ================= */
extern float Max_PCurrent_M1;
extern float Max_PCurrent_M2;
extern I2C_HandleTypeDef hi2c1;

void MotorControl_Init(void);
extern volatile uint32_t pid_isr_count;
/* ================= Calibration / Homing ================= */
/* Maps from:
 * FindMinPositionVoltageMotorX
 * FindMaxPositionVoltageMotorX
 * Update_Home_Positions
 */
float MotorControl_FindMin(Motor_ID_t motor);
float MotorControl_FindMax(Motor_ID_t motor);
void  MotorControl_UpdateHome(void);
void MotorControl_Abort(Motor_ID_t motor);
/* ================= Motion Control ================= */
/* Maps from:
 * moveMotorXToADCValue
 */
void MotorControl_MoveToADC(Motor_ID_t motor,
                            uint16_t targetADC,
                            uint16_t tolerance);

/* ================= Smoothness ================= */
/* Maps from:
 * moveMotorXToADCValue_And_Measure_Smoothness
 */
void MotorControl_MoveAndMeasureSmoothness(
        Motor_ID_t motor,
        uint16_t targetADC,
        uint32_t expected_duration_ms,
        uint16_t tolerance,
        int32_t *max_error_out);

/* ================= Test Helpers ================= */
/* Maps from:
 * ADC_MIN_TO_ADC_MAX_MX
 * ADC_MAX_TO_ADC_MIN_MX
 * Get_MX_Min_to_Max_Smoothness
 * Get_MX_Max_to_Min_Smoothness
 */
uint32_t MotorControl_MinToMax(Motor_ID_t motor,float * P15 , float * N15, float * V24);
uint32_t MotorControl_MaxToMin(Motor_ID_t motor,float * P15 , float * N15, float * V24);

float MotorControl_GetSmoothness_MinToMax(Motor_ID_t motor);
float MotorControl_GetSmoothness_MaxToMin(Motor_ID_t motor);

/* ================= Access ================= */

MotorCalibration_t* MotorControl_GetCalibration(Motor_ID_t motor);
extern uint16_t volatile Pot1_2[2];


#endif
