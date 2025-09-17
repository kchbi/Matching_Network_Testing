/*
 * sine_generator.c
 *
 *  Created on: Sep 17, 2025
 *      Author: AdityaSingh
 */

#include "sine_generator.h"
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define SINE_SAMPLES    256   // Number of samples in one sine wave period
#define DAC_MAX_VAL     4095  // 12-bit DAC maximum value

/* Private variables ---------------------------------------------------------*/
// This table will store the pre-calculated sine wave values.
// 'static' makes it private to this file.
static uint16_t sine_table[SINE_SAMPLES];

/* Private function prototypes -----------------------------------------------*/
// This function is only used inside this file, so we declare it 'static'.
static void SineTable_Init(void);

/**
 * @brief Initializes the sine wave look-up table.
 */
static void SineTable_Init(void)
{
    for (int i = 0; i < SINE_SAMPLES; i++) {
        // Calculate the sine value, scale it to the 12-bit DAC range (0-4095)
        // sinf() -> returns -1.0 to 1.0
        // + 1.0 -> shifts range to 0.0 to 2.0
        // * (DAC_MAX_VAL / 2.0) -> scales range to 0 to 4095
        sine_table[i] = (uint16_t)((sinf(2 * M_PI * i / SINE_SAMPLES) + 1.0) * (DAC_MAX_VAL / 2.0));
    }
}

/**
 * @brief Public function to start the sine wave generation.
 */
void SineGenerator_Start(DAC_HandleTypeDef* hdac, TIM_HandleTypeDef* htim)
{
    // 1. Generate the sine wave look-up table
    SineTable_Init();

    // 2. Start the DAC with DMA transfer
    //    - hdac:         Pointer to the DAC handle
    //    - DAC_CHANNEL_1: The specific DAC channel
    //    - sine_table:    The source of the data (our array)
    //    - SINE_SAMPLES:  The number of data points to send
    //    - DAC_ALIGN_12B_R: Right-align the 12-bit data in a 16-bit register
    HAL_DAC_Start_DMA(hdac, DAC_CHANNEL_1,
                      (uint32_t*)sine_table,
                      SINE_SAMPLES,
                      DAC_ALIGN_12B_R);

    // 3. Start the timer that triggers the DAC
    HAL_TIM_Base_Start(htim);
}
