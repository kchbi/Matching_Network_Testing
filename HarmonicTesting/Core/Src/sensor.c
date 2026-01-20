/*
 * sensor.c
 *
 *  Created on: 12-Jan-2026
 *      Author: AdityaSingh
 */

#include "stm32f4xx_hal.h"
#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include "sensor.h"


volatile bool sensor_state[SENSOR_COUNT] = {0};
volatile SensorEvent_t sensor_event = SENSOR_EVENT_NONE;
volatile bool sensor_event_pending = false ;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    bool new_state;

    /* ---------- SENSOR 1 (PB10) ---------- */
    if (GPIO_Pin == GPIO_PIN_10)
    {
        new_state = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET);

        if (new_state && !sensor_state[SENSOR1])
            sensor_event = SENSOR1_RISE;
        else if (!new_state && sensor_state[SENSOR1])
            sensor_event = SENSOR1_FALL;

        sensor_state[SENSOR1] = new_state;
    }

    /* ---------- SENSOR 2 (PB12) ---------- */
    else if (GPIO_Pin == GPIO_PIN_12)
    {
        new_state = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET);

        if (new_state && !sensor_state[SENSOR2])
            sensor_event = SENSOR2_RISE;
        else if (!new_state && sensor_state[SENSOR2])
            sensor_event = SENSOR2_FALL;

        sensor_state[SENSOR2] = new_state;
    }

    /* ---------- SENSOR 3 (PB13) ---------- */
    else if (GPIO_Pin == GPIO_PIN_13)
    {
        new_state = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_SET);

        if (new_state && !sensor_state[SENSOR3])
            sensor_event = SENSOR3_RISE;
        else if (!new_state && sensor_state[SENSOR3])
            sensor_event = SENSOR3_FALL;

        sensor_state[SENSOR3] = new_state;
    }

    /* ---------- SENSOR 4 (PB14) ---------- */
    else if (GPIO_Pin == GPIO_PIN_14)
    {
        new_state = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET);

        if (new_state && !sensor_state[SENSOR4])
            sensor_event = SENSOR4_RISE;
        else if (!new_state && sensor_state[SENSOR4])
            sensor_event = SENSOR4_FALL;

        sensor_state[SENSOR4] = new_state;
    }

    /* ---------- SENSOR 5 (PB15) ---------- */
    else if (GPIO_Pin == GPIO_PIN_15)
    {
        new_state = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_SET);

        if (new_state && !sensor_state[SENSOR5])
            sensor_event = SENSOR5_RISE;
        else if (!new_state && sensor_state[SENSOR5])
            sensor_event = SENSOR5_FALL;

        sensor_state[SENSOR5] = new_state;
    }
}
