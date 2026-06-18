/*
 * sine_generator.h
 *
 *  Created on: Sep 17, 2025
 *      Author: AdityaSingh
 */

#ifndef INC_DAC_GENERATOR_H_
#define INC_DAC_GENERATOR_H_

#define DAC_MAX_VAL     4095  // 12-bit DAC maximum value
#define DAC_MIN_VAL 0

#include "stm32f4xx_hal.h"
/**
 * @brief Starts the sine wave generation.
 * @note This function initializes the sine look-up table and starts the
 *       DAC, DMA, and Timer peripherals.
 * @param hdac Pointer to the DAC handle.
 * @param htim Pointer to the Timer handle that provides the trigger.
 * @retval None
 */
void DACGenerator_Start(DAC_HandleTypeDef *hdac, uint32_t channel, float voltage, float vref );
void DACGenerator_Stop(DAC_HandleTypeDef* hdac, uint32_t channel);
#endif /* INC_DAC_GENERATOR_H_ */
