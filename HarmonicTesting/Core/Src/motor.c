/*
 * stepper.c
 *
 *  Created on: 07-Jan-2026
 *      Author: AdityaSingh
 */

#include "stm32f4xx_hal.h"
#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include "motor.h"



void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim -> Instance != TIM2) //Checks if the Interrupt is coming from TIM2 or not
	{
		return;

	}
	if (htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if((motors[0].motorstate == MOVING) && (motors[0].toggles_remaining > 0) )
		{
			HAL_GPIO_TogglePin(motors[0].step_port,motors[0].step_pin);
			motors[0].toggles_remaining = motors[0].toggles_remaining - 1 ;
			if (motors[0].toggles_remaining == 0)
			{
				motors[0].motorstate = IDLE ;
			}
		}
	}
	else if (htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		if((motors[1].motorstate == MOVING) && (motors[1].toggles_remaining > 0) )
				{
					HAL_GPIO_TogglePin(motors[1].step_port,motors[1].step_pin);
					motors[1].toggles_remaining = motors[1].toggles_remaining - 1 ;
					if (motors[1].toggles_remaining == 0)
					{
						motors[1].motorstate = IDLE ;
					}
				}

	}
	else if (htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	{
		if((motors[2].motorstate == MOVING) && (motors[2].toggles_remaining > 0) )
				{
					HAL_GPIO_TogglePin(motors[2].step_port,motors[2].step_pin);
					motors[2].toggles_remaining = motors[2].toggles_remaining - 1 ;
					if (motors[2].toggles_remaining == 0)
					{
						motors[2].motorstate = IDLE ;
					}
				}
	}
}

void motor_start_move(uint8_t motor_id ,MotorDirection_t direction,uint32_t toggles)
{
	if (motors[motor_id].motorstate == MOVING)
	{
		return;
	}

	if (direction == DIR_CW)
	{
		HAL_GPIO_WritePin(motors[motor_id].dir_port , motors[motor_id].dir_pin , GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(motors[motor_id].dir_port , motors[motor_id].dir_pin , GPIO_PIN_RESET);

	}
	motors[motor_id].toggles_remaining = toggles;
	motors[motor_id].motorstate = MOVING;

}
void motor_stop(uint8_t motor_id){
	motors[motor_id].toggles_remaining = 0 ;
}

bool motor_is_busy(uint8_t motor_id){
	if (motors[motor_id].motorstate == IDLE){
		return false ;
	}
	return true;
}
