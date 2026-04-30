/*
 * I2ccomm.h
 *
 *  Created on: Jul 2, 2025
 *      Author: adity
 */

#ifndef INC_I2CCOMM_H_
#define INC_I2CCOMM_H_

#include "stm32f4xx_hal.h"

#define T15VP      (0x40)
#define T24V       (0x45)
#define Pot1       (0x4C)
#define Pot2       (0x48)

#define MASK_ENABLE_REGISTER  0x06
#define CVRF_BIT              (1 << 3)



// --- Public Function Prototypes ---
extern uint16_t DevAddress;

// Initializes the INA226 sensor with a given I2C handle and shunt resistor value.
HAL_StatusTypeDef I2C_Init_INA226(I2C_HandleTypeDef *hi2c,
                                 uint16_t DevAddress,
                                 uint16_t config,
                                 uint16_t calibration);

// Reads the bus voltage in Volts.
HAL_StatusTypeDef I2C_ReadVoltage(I2C_HandleTypeDef *hi2c, uint16_t DevAddress , float *Voltage);

// Reads the current in Amperes.
HAL_StatusTypeDef I2C_ReadCurrent(I2C_HandleTypeDef *hi2c, uint16_t DevAddress , float *Current);

HAL_StatusTypeDef I2C_ReadShuntVoltage(I2C_HandleTypeDef *hi2c, uint16_t DevAddress , float *Voltage);

void I2C_ClearBus(I2C_HandleTypeDef *hi2c);

void I2C_Scan(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef I2C_WaitForConversionPolling(I2C_HandleTypeDef *hi2c,
                                               uint16_t DevAddress,
                                               uint32_t avg_count,
                                               float conv_time_ms_per_sample);

#endif /* INC_I2CCOMM_H_ */
