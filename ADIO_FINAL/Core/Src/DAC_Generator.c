/*
 * sine_generator.c
 *
 *  Created on: Sep 17, 2025
 *      Author: AdityaSingh
 */

#include <DAC_Generator.h>
#include "main.h"

/* Private defines -----------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
// This table will store the pre-calculated sine wave values.
// 'static' makes it private to this file.
/* Private function prototypes -----------------------------------------------*/
// This function is only used inside this file, so we declare it 'static'.

void DACGenerator_Start(DAC_HandleTypeDef *hdac, uint32_t channel, uint16_t command )
{
    // If that channel was previously running with DMA, stop it first
    // (safe-guard — HAL_DAC_Stop will also work when no DMA was used)
    HAL_DAC_Stop(hdac, channel);
    // Start only that channel and set the value
    HAL_DAC_SetValue(hdac, channel, DAC_ALIGN_12B_R, command);
    HAL_DAC_Start(hdac, channel);

}


void DACGenerator_Stop(DAC_HandleTypeDef* hdac, uint32_t channel)
{
	HAL_DAC_Stop(hdac, channel);
}
