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
extern volatile bool sensor_state[SENSOR_COUNT];

typedef enum {
	SENSOR1,
	SENSOR2,
	SENSOR3,
	SENSOR4,
	SENSOR5
}SensorID;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);











#endif /* APPLICATION_USER_CORE_SENSOR_H_ */
