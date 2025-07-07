/*
 * ina226.h
 *
 *  Created on: Jul 3, 2025
 *      Author: adity
 */

#ifndef INC_INA226_H_
#define INC_INA226_H_
#include "main.h" // Includes the correct HAL drivers for your board

/*================================================================================================================_*/
/*                                                                                                                */
/*                                             CONFIGURATION MACROS                                               */
/*                                                                                                                */
/*================================================================================================================_*/

// INA226 I2C Address (dependent on A0, A1 pin connections)
#define INA226_I2C_ADDR_GND 0x40 // A0 = GND, A1 = GND
#define INA226_I2C_ADDR_VS  0x41 // A0 = VS,  A1 = GND
#define INA226_I2C_ADDR_SDA 0x44 // A0 = SDA, A1 = GND
#define INA226_I2C_ADDR_SCL 0x45 // A0 = SCL, A1 = GND

/*================================================================================================================_*/
/*                                                                                                                */
/*                                          DRIVER CONFIGURATION STRUCT                                           */
/*                                                                                                                */
/*================================================================================================================_*/

/**
 * @brief  INA226 Driver Handle Structure
 * @note   This structure holds all the configuration and state for a single INA226 device.
 */
typedef struct {
    I2C_HandleTypeDef*  i2c_handle;            // Pointer to the I2C peripheral handle
    uint16_t            device_address;        // I2C address of the device

    // Configuration settings
    uint16_t            config_register;       // Holds the value for the configuration register

    // Calibration values
    float               shunt_resistor_ohm;    // Value of the shunt resistor in Ohms
    float               max_expected_amps;     // Max current expected, used for calibration
    uint16_t            calibration_value;     // Calculated calibration register value

    // LSB values for converting raw data
    float               current_lsb;           // The value of 1 bit of the Current Register in Amperes
    float               power_lsb;             // The value of 1 bit of the Power Register in Watts

} INA226_Handle_t;


/*================================================================================================================_*/
/*                                                                                                                */
/*                                            PUBLIC FUNCTION PROTOTYPES                                          */
/*                                                                                                                */
/*================================================================================================================_*/

/**
 * @brief  Initializes the INA226 sensor.
 * @param  dev: Pointer to the INA226 handle structure.
 * @param  i2c_handle: Pointer to the I2C peripheral handle.
 * @param  device_address: The 7-bit I2C address of the INA226.
 * @param  shunt_resistor: The value of the shunt resistor in Ohms.
 * @param  max_current: The maximum expected current in Amperes.
 * @retval HAL_StatusTypeDef: HAL_OK if initialization is successful.
 */
HAL_StatusTypeDef INA226_Init(INA226_Handle_t* dev, I2C_HandleTypeDef* i2c_handle, uint16_t device_address, float shunt_resistor, float max_current);

/**
 * @brief  Reads the bus voltage (the voltage on the VBUS pin).
 * @param  dev: Pointer to the INA226 handle structure.
 * @param  voltage: Pointer to a float where the voltage in Volts will be stored.
 * @retval HAL_StatusTypeDef: HAL_OK if read is successful.
 */
HAL_StatusTypeDef INA226_ReadBusVoltage(INA226_Handle_t* dev, float* voltage);

/**
 * @brief  Reads the shunt voltage (voltage drop across the shunt resistor).
 * @param  dev: Pointer to the INA226 handle structure.
 * @param  voltage: Pointer to a float where the voltage in Volts will be stored.
 * @retval HAL_StatusTypeDef: HAL_OK if read is successful.
 */
HAL_StatusTypeDef INA226_ReadShuntVoltage(INA226_Handle_t* dev, float* voltage);

/**
 * @brief  Reads the calculated current flowing through the shunt resistor.
 * @param  dev: Pointer to the INA226 handle structure.
 * @param  current: Pointer to a float where the current in Amperes will be stored.
 * @retval HAL_StatusTypeDef: HAL_OK if read is successful.
 */
HAL_StatusTypeDef INA226_ReadCurrent(INA226_Handle_t* dev, float* current);

/**
 * @brief  Reads the calculated power being delivered to the load.
 * @param  dev: Pointer to the INA226 handle structure.
 * @param  power: Pointer to a float where the power in Watts will be stored.
 * @retval HAL_StatusTypeDef: HAL_OK if read is successful.
 */
HAL_StatusTypeDef INA226_ReadPower(INA226_Handle_t* dev, float* power);


#endif /* INC_INA226_H_ */
