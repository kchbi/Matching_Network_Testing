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

void DACGenerator_Start(DAC_HandleTypeDef *hdac,
                      uint32_t channel,
                      uint16_t command)
{
    if (command > 4095U)
        command = 4095U;

    HAL_DAC_SetValue(hdac,
                     channel,
                     DAC_ALIGN_12B_R,
                     command);
}


void DACGenerator_Stop(DAC_HandleTypeDef* hdac, uint32_t channel)
{
	HAL_DAC_Stop(hdac, channel);
}
