/*
 * test.h
 *
 *  Created on: 20-Jan-2026
 *      Author: AdityaSingh
 */

#ifndef INC_TEST_H_
#define INC_TEST_H_

#include <stdint.h>
#include "motor_driver.h"
#include "stm32f4xx_hal.h"


/* ===== Test control ===== */

#define NUM_PARAMETERS 24
extern float parameters[NUM_PARAMETERS] ;

typedef enum {
    PARAM_X1_TIME_MIN_MAX = 0,
    PARAM_X1_TIME_MAX_MIN = 1,
    PARAM_X2_TIME_MIN_MAX = 2,
    PARAM_X2_TIME_MAX_MIN = 3,
    PARAM_X1_P15V_I_MIN_MAX = 4,
	PARAM_X1_N15V_I_MIN_MAX = 5,
	PARAM_X1_24V_I_MIN_MAX = 6,
    PARAM_X1_P15V_I_MAX_MIN = 7,
	PARAM_X1_N15V_I_MAX_MIN = 8,
	PARAM_X1_24V_I_MAX_MIN = 9,
    PARAM_X2_P15V_I_MIN_MAX = 10,
	PARAM_X2_N15V_I_MIN_MAX = 11,
	PARAM_X2_24V_I_MIN_MAX = 12,
    PARAM_X2_P15V_I_MAX_MIN = 13,
	PARAM_X2_N15V_I_MAX_MIN = 14,
	PARAM_X2_24V_I_MAX_MIN = 15,
	PARAM_X1_MIN_POS_V = 16,
	PARAM_X1_MAX_POS_V = 17,
	PARAM_X2_MIN_POS_V = 18,
	PARAM_X2_MAX_POS_V = 19,
	PARAM_X1_STEP_MIN_MAX=20,
	PARAM_X1_STEP_MAX_MIN=21,
	PARAM_X2_STEP_MIN_MAX = 22,
	PARAM_X2_STEP_MAX_MIN = 23
} ParameterIndex_t;

void Test_Init(void);

/* Full automatic matching test */
void Test_Run(void);

/* Corner positioning */
void Test_GotoCorner(uint8_t corner_id);

void Debug_RunMotorConstantPWM(void);
void Stop_test(void);

#endif /* INC_TEST_H_ */
