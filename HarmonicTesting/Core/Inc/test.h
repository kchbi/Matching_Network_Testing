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
    /* ---------- Idle / Init ---------- */
    FSM_IDLE = 0,

    /* ================= PHASE 1 ================= */

    FSM_MOVE_M1_TO_S1,          // Start rotating Motor-1 towards Sensor-1
    FSM_WAIT_M1_AT_S1,          // Wait for SENSOR1 rising edge

    FSM_SCAN_M2_REGION_1_P1,    // M2 scanning between HOME -> SENSOR3
    FSM_SCAN_M2_REGION_2_P1,    // M2 scanning between SENSOR3 -> SENSOR4
    FSM_SCAN_M2_REGION_3_P1,    // M2 scanning between SENSOR4 -> SENSOR5

    FSM_RETURN_M2_HOME_1,       // Return M2 to home after Phase-1 scan

    /* ================= PHASE 2 ================= */

    FSM_MOVE_M1_TO_S2,          // Start rotating Motor-1 towards Sensor-2
    FSM_WAIT_M1_AT_S2,          // Wait for SENSOR2 rising edge

    FSM_SCAN_M2_REGION_1_P2,    // M2 scanning between HOME -> SENSOR3
    FSM_SCAN_M2_REGION_2_P2,    // M2 scanning between SENSOR3 -> SENSOR4
    FSM_SCAN_M2_REGION_3_P2,    // M2 scanning between SENSOR4 -> SENSOR5

    FSM_RETURN_M2_HOME_2,       // Return M2 to home after Phase-2 scan

    /* ---------- Evaluation ---------- */

    FSM_EVALUATE,               // Validate observed vs expected combos
    FSM_DONE                    // Test complete

} TestFSMState_t;


void test_assembly_init(void);
void test_assembly_run(void);

extern bool test_pass;




#endif /* APPLICATION_USER_CORE_TEST_H_ */
