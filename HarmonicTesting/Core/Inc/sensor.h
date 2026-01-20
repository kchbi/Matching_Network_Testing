/*
 * sensor.h
 *
 *  Created on: 12-Jan-2026
 *      Author: AdityaSingh
 */

#ifndef APPLICATION_USER_CORE_SENSOR_H_
#define APPLICATION_USER_CORE_SENSOR_H_

#include "stm32f4xx_hal_gpio.h"

#define SENSOR_COUNT  5

typedef enum {
	SENSOR1,
	SENSOR2,
	SENSOR3,
	SENSOR4,
	SENSOR5
}SensorID;
typedef enum {
    SENSOR_EVENT_NONE = 0,
    SENSOR1_RISE, SENSOR1_FALL,
    SENSOR2_RISE, SENSOR2_FALL,
    SENSOR3_RISE, SENSOR3_FALL,
    SENSOR4_RISE, SENSOR4_FALL,
    SENSOR5_RISE, SENSOR5_FALL
} SensorEvent_t;

extern volatile bool sensor_state[SENSOR_COUNT];
extern volatile SensorEvent_t sensor_event;

extern volatile bool sensor_event_pending;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);











#endif /* APPLICATION_USER_CORE_SENSOR_H_ */
