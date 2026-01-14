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


volatile bool sensor_state[SENSOR_COUNT] = {false} ;
volatile bool sensor_event_pending = false ;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	sensor_event_pending = true;
	if(GPIO_Pin == GPIO_PIN_10){

		if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10 )) == GPIO_PIN_SET){
			sensor_state[SENSOR1] = true ;
		}
		else{
			sensor_state[SENSOR1] = false ;
		}
	}
	else if(GPIO_Pin == GPIO_PIN_12){

		if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12 )) == GPIO_PIN_SET){
			sensor_state[SENSOR2] = true ;
		}
		else{
			sensor_state[SENSOR2] = false ;
		}
	}
	else if(GPIO_Pin == GPIO_PIN_13){

		if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13 )) == GPIO_PIN_SET){
			sensor_state[SENSOR3] = true ;
		}
		else{
			sensor_state[SENSOR3] = false ;
		}
	}
	else if(GPIO_Pin == GPIO_PIN_14){

		if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14 )) == GPIO_PIN_SET){
			sensor_state[SENSOR4] = true ;
		}
		else{
			sensor_state[SENSOR4] = false ;
		}
	}
	else if(GPIO_Pin == GPIO_PIN_15){

		if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15 )) == GPIO_PIN_SET){
			sensor_state[SENSOR5] = true;
		}
		else{
			sensor_state[SENSOR5] = false;
		}
	}

}
