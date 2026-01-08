/*
 * stepper.c
 *
 *  Created on: 07-Jan-2026
 *      Author: AdityaSingh
 */

#include "stm32f4xx_hal.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <stdint.h>
#include "motor.h"

motor_start_move(uint8_t motor_id ,MotorDirection_t direction,uint32_t toggles){
	if (motors[motor_id].motorstate == MOVING){
		printf("Motor has not completed the Previous Command\r\n");
		return;
	}

	if (direction == DIR_CW){
		HAL_GPIO_WritePin(motors[motor_id].dir_port , motors[motor_id].dir_pin , GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(motors[motor_id].dir_port , motors[motor_id].dir_pin , GPIO_PIN_RESET);

	}
	motors[motor_id].toggles_remaining = toggles;
	motors[motor_id].motorstate = MOVING;


}

