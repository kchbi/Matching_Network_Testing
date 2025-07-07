/*
 * ina226.c
 *
 *  Created on: Jul 3, 2025
 *      Author: adity
 */


#include "ina226.h"

/*================================================================================================================_*/
/*                                                                                                                */
/*                                                  PRIVATE DEFINES                                               */
/*                                                                                                                */
/*================================================================================================================_*/

// Register Addresses
#define INA226_REG_CONFIG          0x00
#define INA226_REG_SHUNT_VOLTAGE   0x01
#define INA226_REG_BUS_VOLTAGE     0x02
#define INA226_REG_POWER           0x03
#define INA226_REG_CURRENT         0x04
#define INA226_REG_CALIBRATION     0x05
#define INA226_REG_MASK_ENABLE     0x06
#define INA226_REG_ALERT_LIMIT     0x07
#define INA226_REG_MANUFACTURER_ID 0xFE
#define INA226_REG_DIE_ID          0xFF

// Configuration Register Bit Definitions
#define INA226_AVG_1               (0x0000)
#define INA226_AVG_4               (0x0200)
#define INA226_AVG_16              (0x0400)
#define INA226_AVG_64              (0x0600)

#define INA226_VBUS_1100us         (0x0100)
#define INA226_VSH_1100us          (0x0020)

#define INA226_MODE_SHUNT_BUS_CONTINUOUS (0x0007)

// Fixed LSB values from datasheet
#define INA226_BUS_VOLTAGE_LSB     0.00125f  // 1.25 mV per LSB
#define INA226_SHUNT_VOLTAGE_LSB   0.0000025f // 2.5 uV per LSB

// Internal fixed value for calibration calculation
#define INA226_CALIB_CONST         0.00512f

/*================================================================================================================_*/
/*                                                                                                                */
/*                                           PRIVATE FUNCTION PROTOTYPES                                          */
/*                                                                                                                */
/*================================================================================================================_*/

static HAL_StatusTypeDef INA226_WriteRegister(INA226_Handle_t* dev, uint8_t reg, uint16_t value);
static HAL_StatusTypeDef INA226_ReadRegister(INA226_Handle_t* dev, uint8_t reg, uint16_t* value);

/*================================================================================================================_*/
/*                                                                                                                */
/*                                            PUBLIC FUNCTION DEFINITIONS                                         */
/*                                                                                                                */
/*================================================================================================================_*/

HAL_StatusTypeDef INA226_Init(INA226_Handle_t* dev, I2C_HandleTypeDef* i2c_handle, uint16_t device_address, float shunt_resistor, float max_current)
{
    // Store configuration in the handle
    dev->i2c_handle = i2c_handle;
    dev->device_address = (device_address << 1); // Use 8-bit address for HAL
    dev->shunt_resistor_ohm = shunt_resistor;
    dev->max_expected_amps = max_current;

    // --- 1. Calculate Calibration Values ---
    // Calculate Current_LSB
    dev->current_lsb = dev->max_expected_amps / 32768.0f;

    // Calculate Power_LSB (25 times Current_LSB)
    dev->power_lsb = 25.0f * dev->current_lsb;

    // Calculate Calibration Register Value
    dev->calibration_value = (uint16_t)(INA226_CALIB_CONST / (dev->current_lsb * dev->shunt_resistor_ohm));


    // --- 2. Configure the Device ---
    // Set a default configuration: 16 averages, 1.1ms conversion time for bus and shunt, continuous mode
    dev->config_register = INA226_AVG_16 | INA226_VBUS_1100us | INA226_VSH_1100us | INA226_MODE_SHUNT_BUS_CONTINUOUS;

    // Write configuration to the register
    if (INA226_WriteRegister(dev, INA226_REG_CONFIG, dev->config_register) != HAL_OK) {
        return HAL_ERROR;
    }

    // Write calibration value to the register
    if (INA226_WriteRegister(dev, INA226_REG_CALIBRATION, dev->calibration_value) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef INA226_ReadBusVoltage(INA226_Handle_t* dev, float* voltage)
{
    uint16_t raw_voltage;

    if (INA226_ReadRegister(dev, INA226_REG_BUS_VOLTAGE, &raw_voltage) != HAL_OK) {
        return HAL_ERROR;
    }

    *voltage = (float)raw_voltage * INA226_BUS_VOLTAGE_LSB;
    return HAL_OK;
}

HAL_StatusTypeDef INA226_ReadShuntVoltage(INA226_Handle_t* dev, float* voltage)
{
    uint16_t raw_voltage_u;
    int16_t raw_voltage_s;

    if (INA226_ReadRegister(dev, INA226_REG_SHUNT_VOLTAGE, &raw_voltage_u) != HAL_OK) {
        return HAL_ERROR;
    }

    // Shunt voltage can be negative
    raw_voltage_s = (int16_t)raw_voltage_u;

    *voltage = (float)raw_voltage_s * INA226_SHUNT_VOLTAGE_LSB;
    return HAL_OK;
}

HAL_StatusTypeDef INA226_ReadCurrent(INA226_Handle_t* dev, float* current)
{
    uint16_t raw_current_u;
    int16_t raw_current_s;

    if (INA226_ReadRegister(dev, INA226_REG_CURRENT, &raw_current_u) != HAL_OK) {
        return HAL_ERROR;
    }

    // Current can be negative
    raw_current_s = (int16_t)raw_current_u;

    *current = (float)raw_current_s * dev->current_lsb;
    return HAL_OK;
}

HAL_StatusTypeDef INA226_ReadPower(INA226_Handle_t* dev, float* power)
{
    uint16_t raw_power;

    if (INA226_ReadRegister(dev, INA226_REG_POWER, &raw_power) != HAL_OK) {
        return HAL_ERROR;
    }

    *power = (float)raw_power * dev->power_lsb;
    return HAL_OK;
}

/*================================================================================================================_*/
/*                                                                                                                */
/*                                           PRIVATE FUNCTION DEFINITIONS                                         */
/*                                                                                                                */
/*================================================================================================================_*/

/**
 * @brief  Writes a 16-bit value to a register on the INA226.
 */
static HAL_StatusTypeDef INA226_WriteRegister(INA226_Handle_t* dev, uint8_t reg, uint16_t value)
{
    uint8_t tx_buffer[2];
    tx_buffer[0] = (uint8_t)(value >> 8);   // MSB
    tx_buffer[1] = (uint8_t)(value & 0xFF); // LSB

    return HAL_I2C_Mem_Write(dev->i2c_handle, dev->device_address, reg, I2C_MEMADD_SIZE_8BIT, tx_buffer, 2, HAL_MAX_DELAY);
}

/**
 * @brief  Reads a 16-bit value from a register on the INA226.
 */
static HAL_StatusTypeDef INA226_ReadRegister(INA226_Handle_t* dev, uint8_t reg, uint16_t* value)
{
    uint8_t rx_buffer[2];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(dev->i2c_handle, dev->device_address, reg, I2C_MEMADD_SIZE_8BIT, rx_buffer, 2, HAL_MAX_DELAY);

    if (status == HAL_OK) {
        *value = ((uint16_t)rx_buffer[0] << 8) | rx_buffer[1];
    }

    return status;
}
