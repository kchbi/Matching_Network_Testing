/*
 * test.h
 *
 *  Created on: 12-Jan-2026
 *      Author: AdityaSingh
 */

#ifndef APPLICATION_USER_CORE_TEST_H_
#define APPLICATION_USER_CORE_TEST_H_
#include "stm32f4xx_hal.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <stdint.h>

/* ---------------- FSM Definition ---------------- */
typedef enum
{
    FSM_IDLE = 0,

    FSM_MOVE_M1_TO_S1,
    FSM_START_SCAN_M2_PHASE1,
    FSM_SCAN_M2_PHASE1,
    FSM_RETURN_M2_HOME_1,

    FSM_MOVE_M1_TO_S2,
    FSM_START_SCAN_M2_PHASE2,
    FSM_SCAN_M2_PHASE2,
    FSM_RETURN_M2_HOME_2,

    FSM_EVALUATE,
    FSM_DONE

} TestFSMState_t;

void test_assembly_init(void);
void test_assembly_run(void);

extern bool test_pass;




#endif /* APPLICATION_USER_CORE_TEST_H_ */
