/*
 * motor_driver.h
 *
 *  Created on: 20-Jan-2026
 *      Author: AdityaSingh
 */

#ifndef INC_MOTOR_DRIVER_H_
#define INC_MOTOR_DRIVER_H_


#include <stdint.h>
#include "stm32f4xx_hal.h"

#define MOTOR_GPIO_PORT GPIOE
#define MOTOR1_IN1          GPIO_PIN_7
#define MOTOR1_IN2          GPIO_PIN_8
#define MOTOR2_IN1          GPIO_PIN_9
#define MOTOR2_IN2          GPIO_PIN_10


typedef enum {
    MOTOR_1 = 0,
    MOTOR_2
} Motor_ID_t;

void Motor_Init(void);
void Motor_Set(Motor_ID_t motor, float target_cmd);
void Motor_Stop(Motor_ID_t motor);
void Motor_StopAll(void);

void Motor_RampUpdate(Motor_ID_t motor);


#endif /* INC_MOTOR_DRIVER_H_ */
