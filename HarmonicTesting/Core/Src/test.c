/*
 * test.c
 *
 *  Created on: 12-Jan-2026
 *      Author: AdityaSingh
 */

#include "motor.h"
#include "sensor.h"
#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "test.h"

/* ---------------- Motor IDs ---------------- */
#define MOTOR_ROTATION   0
#define MOTOR_EXTENSION  1

/* ---------------- Parameters ---------------- */
#define NUM_PARAMETERS 20
float parameters[NUM_PARAMETERS] = {0.0f};

/* ---------------- Expected Sensor Combos ---------------- */
/* Bit position corresponds to SENSOR index */
static const uint8_t expected_phase1[] = {
    0b00011,
    0b00101,
    0b00110
};

static const uint8_t expected_phase2[] = {
    0b01001,
    0b01010,
    0b01100
};

#define PHASE1_COUNT (sizeof(expected_phase1) / sizeof(expected_phase1[0]))
#define PHASE2_COUNT (sizeof(expected_phase2) / sizeof(expected_phase2[0]))

static bool observed_phase1[PHASE1_COUNT];
static bool observed_phase2[PHASE2_COUNT];



static TestFSMState_t fsm_state = FSM_IDLE;
bool test_pass = true;

/* ---------------- Helper: Build Sensor Mask ---------------- */
static uint8_t build_sensor_mask(void)
{
    uint8_t mask = 0;

    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        if (sensor_state[i])
            mask |= (1U << i);
    }
    return mask;
}

/* ---------------- Test Init ---------------- */
void test_assembly_init(void)
{
    fsm_state = FSM_MOVE_M1_TO_S1;
    test_pass = true;

    for (uint8_t i = 0; i < PHASE1_COUNT; i++)
        observed_phase1[i] = false;

    for (uint8_t i = 0; i < PHASE2_COUNT; i++)
        observed_phase2[i] = false;
}

/* ---------------- Test FSM Runner ---------------- */
void test_assembly_run(void)
{
    uint8_t mask;

    switch (fsm_state)
    {
    /* ================= PHASE 1 ================= */

    case FSM_MOVE_M1_TO_S1:
        motor_start_move(MOTOR_ROTATION, DIR_CW, 100000);
        fsm_state = FSM_WAIT_M1_AT_S1;
        break;

    case FSM_WAIT_M1_AT_S1:
        if (sensor_event == SENSOR1_RISE)
        {
            sensor_event = SENSOR_EVENT_NONE;
            motor_stop(MOTOR_ROTATION);

            motor_start_move(MOTOR_EXTENSION, DIR_CW, 100000);
            fsm_state = FSM_SCAN_M2_REGION_1_P1;
        }
        break;

    /* ---------- M2 REGION 1 (P1) ---------- */
    case FSM_SCAN_M2_REGION_1_P1:
        if (sensor_event != SENSOR_EVENT_NONE)
        {
            mask = build_sensor_mask();

            for (uint8_t i = 0; i < PHASE1_COUNT; i++)
                if (mask == expected_phase1[i])
                    observed_phase1[i] = true;

            if (sensor_event == SENSOR3_RISE)
            {
                sensor_event = SENSOR_EVENT_NONE;
                fsm_state = FSM_SCAN_M2_REGION_2_P1;
                break;
            }

            sensor_event = SENSOR_EVENT_NONE;
        }
        break;

    /* ---------- M2 REGION 2 (P1) ---------- */
    case FSM_SCAN_M2_REGION_2_P1:
        if (sensor_event != SENSOR_EVENT_NONE)
        {
            mask = build_sensor_mask();

            for (uint8_t i = 0; i < PHASE1_COUNT; i++)
                if (mask == expected_phase1[i])
                    observed_phase1[i] = true;

            if (sensor_event == SENSOR4_RISE)
            {
                sensor_event = SENSOR_EVENT_NONE;
                fsm_state = FSM_SCAN_M2_REGION_3_P1;
                break;
            }

            sensor_event = SENSOR_EVENT_NONE;
        }
        break;

    /* ---------- M2 REGION 3 (P1) ---------- */
    case FSM_SCAN_M2_REGION_3_P1:
        if (sensor_event != SENSOR_EVENT_NONE)
        {
            mask = build_sensor_mask();

            for (uint8_t i = 0; i < PHASE1_COUNT; i++)
                if (mask == expected_phase1[i])
                    observed_phase1[i] = true;

            if (sensor_event == SENSOR5_RISE)
            {
                sensor_event = SENSOR_EVENT_NONE;
                motor_stop(MOTOR_EXTENSION);
                motor_start_move(MOTOR_EXTENSION, DIR_CCW, 100000);
                fsm_state = FSM_RETURN_M2_HOME_1;
                break;
            }

            sensor_event = SENSOR_EVENT_NONE;
        }
        break;

    case FSM_RETURN_M2_HOME_1:
        if (sensor_event == SENSOR3_RISE)
        {
            sensor_event = SENSOR_EVENT_NONE;
            motor_stop(MOTOR_EXTENSION);
            fsm_state = FSM_MOVE_M1_TO_S2;
        }
        break;

    /* ================= PHASE 2 ================= */

    case FSM_MOVE_M1_TO_S2:
        motor_start_move(MOTOR_ROTATION, DIR_CW, 100000);
        fsm_state = FSM_WAIT_M1_AT_S2;
        break;

    case FSM_WAIT_M1_AT_S2:
        if (sensor_event == SENSOR2_RISE)
        {
            sensor_event = SENSOR_EVENT_NONE;
            motor_stop(MOTOR_ROTATION);

            motor_start_move(MOTOR_EXTENSION, DIR_CW, 100000);
            fsm_state = FSM_SCAN_M2_REGION_1_P2;
        }
        break;

    /* ---------- M2 REGION 1 (P2) ---------- */
    case FSM_SCAN_M2_REGION_1_P2:
        if (sensor_event != SENSOR_EVENT_NONE)
        {
            mask = build_sensor_mask();

            for (uint8_t i = 0; i < PHASE2_COUNT; i++)
                if (mask == expected_phase2[i])
                    observed_phase2[i] = true;

            if (sensor_event == SENSOR3_RISE)
            {
                sensor_event = SENSOR_EVENT_NONE;
                fsm_state = FSM_SCAN_M2_REGION_2_P2;
                break;
            }

            sensor_event = SENSOR_EVENT_NONE;
        }
        break;

    /* ---------- M2 REGION 2 (P2) ---------- */
    case FSM_SCAN_M2_REGION_2_P2:
        if (sensor_event != SENSOR_EVENT_NONE)
        {
            mask = build_sensor_mask();

            for (uint8_t i = 0; i < PHASE2_COUNT; i++)
                if (mask == expected_phase2[i])
                    observed_phase2[i] = true;

            if (sensor_event == SENSOR4_RISE)
            {
                sensor_event = SENSOR_EVENT_NONE;
                fsm_state = FSM_SCAN_M2_REGION_3_P2;
                break;
            }

            sensor_event = SENSOR_EVENT_NONE;
        }
        break;

    /* ---------- M2 REGION 3 (P2) ---------- */
    case FSM_SCAN_M2_REGION_3_P2:
        if (sensor_event != SENSOR_EVENT_NONE)
        {
            mask = build_sensor_mask();

            for (uint8_t i = 0; i < PHASE2_COUNT; i++)
                if (mask == expected_phase2[i])
                    observed_phase2[i] = true;

            if (sensor_event == SENSOR5_RISE)
            {
                sensor_event = SENSOR_EVENT_NONE;
                motor_stop(MOTOR_EXTENSION);
                motor_start_move(MOTOR_EXTENSION, DIR_CCW, 100000);
                fsm_state = FSM_RETURN_M2_HOME_2;
                break;
            }

            sensor_event = SENSOR_EVENT_NONE;
        }
        break;

    case FSM_RETURN_M2_HOME_2:
        if (sensor_event == SENSOR3_RISE)
        {
            sensor_event = SENSOR_EVENT_NONE;
            motor_stop(MOTOR_EXTENSION);
            fsm_state = FSM_EVALUATE;
        }
        break;

    /* ================= EVALUATION ================= */

    case FSM_EVALUATE:
        for (uint8_t i = 0; i < PHASE1_COUNT; i++)
            if (!observed_phase1[i])
                test_pass = false;

        for (uint8_t i = 0; i < PHASE2_COUNT; i++)
            if (!observed_phase2[i])
                test_pass = false;

        fsm_state = FSM_DONE;
        break;

    case FSM_DONE:
        /* Test complete */
        break;

    default:
        break;
    }
}

/* ---------------- Result Transmission ---------------- */
void transmit_json(float *parameters)
{
    printf("DATA,");

    for (int i = 0; i < NUM_PARAMETERS; i++)
    {
        printf("%ld", (int32_t)(parameters[i] * 1000.0f));
        if (i < NUM_PARAMETERS - 1)
            printf(",");
    }

    printf("\n");
    fflush(stdout);
}
