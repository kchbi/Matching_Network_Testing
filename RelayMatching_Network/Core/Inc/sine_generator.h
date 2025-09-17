/*
 * sine_generator.h
 *
 *  Created on: Sep 17, 2025
 *      Author: AdityaSingh
 */

#ifndef INC_SINE_GENERATOR_H_
#define INC_SINE_GENERATOR_H_
/**
 * @brief Starts the sine wave generation.
 * @note This function initializes the sine look-up table and starts the
 *       DAC, DMA, and Timer peripherals.
 * @param hdac Pointer to the DAC handle.
 * @param htim Pointer to the Timer handle that provides the trigger.
 * @retval None
 */
void SineGenerator_Start(DAC_HandleTypeDef* hdac, TIM_HandleTypeDef* htim);


#endif /* INC_SINE_GENERATOR_H_ */
