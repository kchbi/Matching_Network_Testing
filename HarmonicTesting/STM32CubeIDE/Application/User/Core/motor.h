/*
 * stepper.h
 *
 *  Created on: 07-Jan-2026
 *      Author: AdityaSingh
 */

#ifndef APPLICATION_USER_CORE_MOTOR_H_
#define APPLICATION_USER_CORE_MOTOR_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

#define STEPPER_COUNT 3

typedef enum {
	IDLE,
	MOVING
}MotorState_t;

typedef struct {
	volatile int32_t toggles_remaining;
	GPIO_TypeDef *dir_port;
	uint16_t dir_pin;
	GPIO_TypeDef *step_port;
	uint16_t step_pin;
	MotorState_t motorstate;
}Motor_t;


extern Motor_t motors[STEPPER_COUNT];

typedef enum {
	DIR_CW ,
	DIR_CCW
}MotorDirection_t;


void motor_start_move(uint8_t motor_id ,MotorDirection_t direction,uint32_t toggles);
bool motor_is_busy(uint8_t motor_id);
void motor_stop(uint8_t motor_id);

#endif /* APPLICATION_USER_CORE_MOTOR_H_ */
