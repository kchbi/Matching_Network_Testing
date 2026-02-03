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

#define STATE_IDLE        1
#define STATE_CAL_M1_MIN  2
#define STATE_CAL_M1_MAX  3
#define STATE_CAL_M2_MIN  4
#define STATE_CAL_M2_MAX  5

#define NUM_PARAMETERS 4
extern float parameters[NUM_PARAMETERS] ;

typedef enum {

	PARAM_X1_MIN_POS_V = 0,
	PARAM_X1_MAX_POS_V = 1,
	PARAM_X2_MIN_POS_V = 2,
	PARAM_X2_MAX_POS_V = 3,

} ParameterIndex_t;

void Test_Init(void);

/* Full automatic matching test */
void Test_Run(void);

/* Corner positioning */
void Test_GotoCorner(uint8_t corner_id);
void Stop_test(void);

#endif /* INC_TEST_H_ */
