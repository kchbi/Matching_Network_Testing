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
#define MOTOR1_IN1          GPIO_PIN_8
#define MOTOR1_IN2          GPIO_PIN_7
#define MOTOR2_IN1          GPIO_PIN_9
#define MOTOR2_IN2          GPIO_PIN_10

#define PWM_MAX_DUTY     4200
#define PWM_MIN_DUTY     0
typedef enum {
    MOTOR_1 = 0,
    MOTOR_2
} Motor_ID_t;

void Motor_Init(void);
void Motor_Set(Motor_ID_t motor, float command);
void Motor_Stop(Motor_ID_t motor);
void Motor_StopAll(void);




#endif /* INC_MOTOR_DRIVER_H_ */
