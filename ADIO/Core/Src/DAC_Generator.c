/*
 * sine_generator.c
 *
 *  Created on: Sep 17, 2025
 *      Author: AdityaSingh
 */

#include <DAC_Generator.h>
#include <math.h>
#include <stdint.h>
#include "main.h"

/* Private defines -----------------------------------------------------------*/

#define DAC_MAX_VAL     4095  // 12-bit DAC maximum value

/* Private variables ---------------------------------------------------------*/
// This table will store the pre-calculated sine wave values.
// 'static' makes it private to this file.
/* Private function prototypes -----------------------------------------------*/
// This function is only used inside this file, so we declare it 'static'.

void DACGenerator_Start(DAC_HandleTypeDef *hdac, uint32_t channel, float voltage, float vref )
{

    if (vref <= 0.0f) vref = 10.0f;

    // Simple clamp of input voltage to allowable range [0, vref]
    if (voltage < 0.0f) voltage = 0.0f;
    if (voltage > vref) voltage = vref;

    // Convert voltage to 12-bit code (right aligned)
    uint32_t code = (uint32_t)roundf((voltage * 4095.0f) / vref);
    if (code > 4095U) code = 4095U;

    // If that channel was previously running with DMA, stop it first
    // (safe-guard — HAL_DAC_Stop will also work when no DMA was used)
    HAL_DAC_Stop(hdac, channel);

    // Start only that channel and set the value
    HAL_DAC_Start(hdac, channel);
    HAL_DAC_SetValue(hdac, channel, DAC_ALIGN_12B_R, code);
}


void DACGenerator_Stop(DAC_HandleTypeDef* hdac, uint32_t channel)
{
	HAL_DAC_Stop(hdac, channel);
}
