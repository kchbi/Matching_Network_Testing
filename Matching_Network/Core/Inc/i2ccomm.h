/*
 * I2ccomm.h
 *
 *  Created on: Jul 2, 2025
 *      Author: adity
 */

#ifndef INC_I2CCOMM_H_
#define INC_I2CCOMM_H_

#include "stm32f4xx_hal.h"

#define Motor1N      (0x45)
#define Motor1P      (0x40)
#define Motor2N      (0x44)
#define Motor2P      (0x41)
#define Pot1         (0x4C)
#define Pot2         (0x42)


// --- Public Function Prototypes ---
extern uint32_t Timeout;
extern uint16_t DevAddress;

// Initializes the INA226 sensor with a given I2C handle and shunt resistor value.
HAL_StatusTypeDef I2C_Init(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);

// Reads the bus voltage in Volts.
HAL_StatusTypeDef I2C_ReadVoltage(I2C_HandleTypeDef *hi2c, uint16_t DevAddress , float *Voltage);

// Reads the current in Amperes.
HAL_StatusTypeDef I2C_ReadCurrent(I2C_HandleTypeDef *hi2c, uint16_t DevAddress , float *Current);

HAL_StatusTypeDef I2C_ReadShuntVoltage(I2C_HandleTypeDef *hi2c, uint16_t DevAddress , float *Voltage);

void I2C_ClearBus(I2C_HandleTypeDef *hi2c);

void I2C_Scan(I2C_HandleTypeDef *hi2c);

#endif /* INC_I2CCOMM_H_ */
