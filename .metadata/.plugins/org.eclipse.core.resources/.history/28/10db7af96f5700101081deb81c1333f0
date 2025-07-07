/*
 * I2ccomm.h
 *
 *  Created on: Jul 2, 2025
 *      Author: adity
 */

#ifndef INC_I2CCOMM_H_
#define INC_I2CCOMM_H_

#include "stm32f4xx_hal.h"

// --- Public Function Prototypes ---
extern uint32_t Timeout;

// Initializes the INA226 sensor with a given I2C handle and shunt resistor value.
HAL_StatusTypeDef I2C_Init(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, float shunt_resistor_ohm);

// Reads the bus voltage in Volts.
float I2C_ReadBusVoltage(void);

// Reads the current in Amperes.
float I2C_ReadCurrent(void);

#endif /* INC_I2CCOMM_H_ */
