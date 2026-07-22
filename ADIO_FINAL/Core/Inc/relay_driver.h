/*
 * relay_control.h
 *
 *  Created on: 20-Jan-2026
 *      Author: AdityaSingh
 */

#ifndef INC_RELAY_DRIVER_H_
#define INC_RELAY_DRIVER_H_
#include <stdint.h>
#include "stm32f4xx_hal.h"

#define RELAY1_PIN   GPIO_PIN_9
#define RELAY2_PIN   GPIO_PIN_11
#define RELAY3_PIN   GPIO_PIN_12
#define RELAY4_PIN   GPIO_PIN_3

#define RELAY_GPIO_PORT GPIOA

typedef enum {
    RELAY_ON,
    RELAY_OFF
} Relay_State_t;

typedef enum {
	RELAY_1,
	RELAY_2,
	RELAY_3,
	RELAY_4
}Relay_ID_t;

void set_relay_combination(uint8_t combination);
void relay_reset(void) ;
void relay_set_state(Relay_ID_t relayID, Relay_State_t state);

#endif /* INC_RELAY_DRIVER_H_ */
