/*
 * test.h
 *
 *  Created on: 12-Jan-2026
 *      Author: AdityaSingh
 */

#ifndef APPLICATION_USER_CORE_TEST_H_
#define APPLICATION_USER_CORE_TEST_H_
#include "stm32f4xx_hal.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <stdint.h>


extern uint8_t current_mask;

uint8_t ideal_sensor_combos =
{
		0b0000,
		0b0001,
		0b0011,
		0b0111,
		0b0110


};



#endif /* APPLICATION_USER_CORE_TEST_H_ */
