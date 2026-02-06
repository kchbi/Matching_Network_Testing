/*
 * i2ccomm.c
 *
 *  Created on: Jul 2, 2025
 *      Author: adity
 */
#include "i2ccomm.h"
#include <stdio.h>
#include <string.h>
#include "main.h"
//======================================================================
// --- IMPORTANT: CONFIGURE THESE DEFINES FOR YOUR SPECIFIC PINS ---
//=======================================================================
#define I2C_SCL_PIN     GPIO_PIN_6
#define I2C_SDA_PIN     GPIO_PIN_7
#define I2C_GPIO_PORT   GPIOB

//================================================================================
//2.Device Addresses for Current and Voltage Detection in Motors and Potentiometer
//================================================================================


static const uint32_t Timeout = 100;

static const float Maximum_Expected_Current = 0.7f;
//static const float Shunt_Resistor_Value     = 0.1f;
static const float Current_LSB              = Maximum_Expected_Current / 32768.0f;
//static const float base_cal_float           = 0.00512f / (Current_LSB * Shunt_Resistor_Value);
//static const float calratio                 = 1.0f;  // Change this to tune calibration
//static const uint16_t calibration_value     = (uint16_t)(base_cal_float * calratio);  // Truncated

#define ConfigAddress 0x00
#define CalibrationAddress 0x05
#define VoltageAddress 0x02
#define CurrentAddress 0x04
#define ConfigSettings 0x4B7F
#define SHUNT_VOLTAGE_ADDRESS  0x01




/**
 * @brief  Initialize INA226 device with configuration and calibration
 * @note   Performs bus recovery, retry logic, reset, config write,
 *         calibration write, and configuration verification.
 *
 * @param  hi2c        Pointer to I2C handle
 * @param  DevAddress  7-bit I2C device address
 *
 * @retval HAL_OK      Initialization successful
 * @retval HAL_ERROR   Device not responding or verification failed
 */

HAL_StatusTypeDef I2C_Init_INA226(I2C_HandleTypeDef *hi2c,
                                 uint16_t DevAddress,
                                 uint16_t config,
                                 uint16_t calibration)
{
    HAL_StatusTypeDef status;
    uint8_t tx_buffer[2];
    uint8_t rx_buffer[2];

    /* 1. Ensure bus is free */
    if (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY)
    {
        I2C_ClearBus(hi2c);
    }

    /* 2. Check device readiness */
    for (uint8_t attempt = 0; attempt < 3; attempt++)
    {
        status = HAL_I2C_IsDeviceReady(hi2c,
                                       (DevAddress << 1),
                                       3,
                                       Timeout);
        if (status == HAL_OK)
            break;

        if (attempt == 1)
            I2C_ClearBus(hi2c);
    }

    if (status != HAL_OK)
        return HAL_ERROR;

    /* 3. Reset INA226 */
    tx_buffer[0] = 0x80;
    tx_buffer[1] = 0x00;

    status = HAL_I2C_Mem_Write(hi2c,
                               (DevAddress << 1),
                               ConfigAddress,
                               I2C_MEMADD_SIZE_8BIT,
                               tx_buffer,
                               2,
                               Timeout);
    if (status != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(3);

    /* 4. Write configuration register */
    tx_buffer[0] = (uint8_t)(config >> 8);
    tx_buffer[1] = (uint8_t)(config & 0xFF);

    status = HAL_I2C_Mem_Write(hi2c,
                               (DevAddress << 1),
                               ConfigAddress,
                               I2C_MEMADD_SIZE_8BIT,
                               tx_buffer,
                               2,
                               Timeout);
    if (status != HAL_OK)
        return HAL_ERROR;

    /* 5. Write calibration register */
    tx_buffer[0] = (uint8_t)(calibration >> 8);
    tx_buffer[1] = (uint8_t)(calibration & 0xFF);

    status = HAL_I2C_Mem_Write(hi2c,
                               (DevAddress << 1),
                               CalibrationAddress,
                               I2C_MEMADD_SIZE_8BIT,
                               tx_buffer,
                               2,
                               Timeout);
    if (status != HAL_OK)
        return HAL_ERROR;

    /* 6. Verify configuration */
    status = HAL_I2C_Mem_Read(hi2c,
                              (DevAddress << 1),
                              ConfigAddress,
                              I2C_MEMADD_SIZE_8BIT,
                              rx_buffer,
                              2,
                              Timeout);
    if (status != HAL_OK)
        return HAL_ERROR;

    uint16_t actual_config = ((uint16_t)rx_buffer[0] << 8) | rx_buffer[1];
    if (actual_config != config)
        return HAL_ERROR;

    return HAL_OK;
}


HAL_StatusTypeDef I2C_ReadVoltage(I2C_HandleTypeDef *hi2c,
                                  uint16_t DevAddress,
                                  float *Voltage)
{
    if (Voltage == NULL)
        return HAL_ERROR;

    uint8_t rx_buffer[2];
    HAL_StatusTypeDef status;
    uint16_t raw_voltage;

    /* Retry once to handle INA226 NACK / clock stretching */
    for (uint8_t attempt = 0; attempt < 2; attempt++)
    {
        status = HAL_I2C_Mem_Read(hi2c,
                                  (DevAddress << 1)+1,
                                  VoltageAddress,
                                  I2C_MEMADD_SIZE_8BIT,
                                  rx_buffer,
                                  2,
                                  Timeout);

        if (status == HAL_OK)
            break;

        /* Attempt I2C recovery on first failure */
        if (attempt == 0)
        {
            I2C_ClearBus(hi2c);
        }
    }

    if (status != HAL_OK)
        return HAL_ERROR;

    /* Combine MSB and LSB */
    raw_voltage = (uint16_t)((rx_buffer[0] << 8) | rx_buffer[1]);

    /* INA226 sanity check:
       - 0x0000 often appears during startup or conversion gaps */
    if (raw_voltage == 0x0000)
    {
        return HAL_ERROR;
    }

    /* Convert to Volts (1.25 mV per bit) */
    *Voltage = (float)raw_voltage * 0.00125f;

    return HAL_OK;
}

/**
 * @brief  Read current from INA226
 * @note   Includes retry logic, bus recovery, and overflow detection
 *
 * @param  hi2c        Pointer to I2C handle
 * @param  DevAddress  7-bit I2C address
 * @param  Current     Output current in Amperes
 *
 * @retval HAL_OK
 * @retval HAL_ERROR
 */

HAL_StatusTypeDef I2C_ReadCurrent(I2C_HandleTypeDef *hi2c,
                                  uint16_t DevAddress,
                                  float *Current)
{
    if (Current == NULL)
        return HAL_ERROR;

    uint8_t rx_buffer[2];
    HAL_StatusTypeDef status;
    int16_t raw_current;

    /* Try read twice (handles transient INA226 NACKs) */
    for (uint8_t attempt = 0; attempt < 2; attempt++)
    {
        status = HAL_I2C_Mem_Read(hi2c,
                                  (DevAddress << 1)+1,
                                  CurrentAddress,
                                  I2C_MEMADD_SIZE_8BIT,
                                  rx_buffer,
                                  2,
                                  Timeout);

        if (status == HAL_OK)
            break;

        /* Attempt bus recovery once */
        if (attempt == 0)
        {
            I2C_ClearBus(hi2c);
        }
    }

    if (status != HAL_OK)
        return HAL_ERROR;

    /* Combine bytes (MSB first) */
    raw_current = (int16_t)((rx_buffer[0] << 8) | rx_buffer[1]);

    /* Convert to Amperes */
    *Current = (float)raw_current * Current_LSB;

    return HAL_OK;
}

HAL_StatusTypeDef I2C_ReadShuntVoltage(I2C_HandleTypeDef *hi2c,
                                      uint16_t DevAddress,
                                      float *Voltage)
{
    if (Voltage == NULL)
        return HAL_ERROR;

    uint8_t rx_buffer[2];
    HAL_StatusTypeDef status;
    int16_t raw;

    /* Try read twice (handles transient NACK / clock stretch) */
    for (uint8_t attempt = 0; attempt < 2; attempt++)
    {
        status = HAL_I2C_Mem_Read(hi2c,
                                  (DevAddress << 1),
								  SHUNT_VOLTAGE_ADDRESS,
                                  I2C_MEMADD_SIZE_8BIT,
                                  rx_buffer,
                                  2,
                                  Timeout);

        if (status == HAL_OK)
            break;

        /* Attempt bus recovery only once */
        if (attempt == 0)
        {
            I2C_ClearBus(hi2c);
        }
    }

    if (status != HAL_OK)
        return HAL_ERROR;

    raw = (int16_t)((rx_buffer[0] << 8) | rx_buffer[1]);

    /* INA226 sanity checks */
    if (raw == 0x7FFF || raw == (int16_t)0x8000)
    {
        return HAL_ERROR;   // invalid / overflow reading
    }

    /* Convert to volts: 2.5 µV per LSB */
    *Voltage = (float)raw * 2.5e-6f;

    return HAL_OK;
}


void I2C_Scan(I2C_HandleTypeDef *hi2c)
{
    printf("--- Starting I2C Bus Scan ---\n");
    HAL_StatusTypeDef res;
    uint8_t devices_found = 0; // Use a counter

    for(uint16_t i = 1; i < 128; i++) // Start scan from 1
    {
        res = HAL_I2C_IsDeviceReady(hi2c, (i << 1), 2, 50); // 2 trials, 10ms timeout
        if(res == HAL_OK)
        {
            printf("--> SUCCESS: I2C Device Found at Address: 0x%02X\n", i);
            devices_found++;
        }
    }

    if (devices_found == 0)
    {
        printf("--> FAILED: No I2C devices were found on this bus.\n");
    }
    printf("--- Scan Finished ---\n\n");
}

/**
  * @brief  Recovers a stuck I2C bus by manually toggling the SCL line.
  * @note   This function should be called at startup, before the main I2C peripheral
  *         initialization, if there's a risk of a slave device holding the bus low.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *               the configuration information for the specified I2C.
  * @retval None
  */
void I2C_ClearBus(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_I2C_DeInit(hi2c);

    GPIO_InitStruct.Pin = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);

    if (HAL_GPIO_ReadPin(I2C_GPIO_PORT, I2C_SDA_PIN) == GPIO_PIN_RESET)
    {
        printf("I2C SDA stuck low, pulsing SCL...\n");
        for (int i = 0; i < 16; i++)
        {
            HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
            for (volatile int d = 0; d < 100; d++);
            HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
            for (volatile int d = 0; d < 100; d++);
        }
    }

    /* Ensure SCL released */
    HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    for (volatile int d = 0; d < 100; d++);

    /* Generate STOP */
    HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SDA_PIN, GPIO_PIN_RESET);
    for (volatile int d = 0; d < 100; d++);
    HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    for (volatile int d = 0; d < 100; d++);
    HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SDA_PIN, GPIO_PIN_SET);

    if (HAL_GPIO_ReadPin(I2C_GPIO_PORT, I2C_SDA_PIN) == GPIO_PIN_RESET)
    {
        printf("I2C bus still stuck after recovery\n");
    }

    if (HAL_I2C_Init(hi2c) != HAL_OK)
    {
        printf("I2C re-init failed\n");
        return;
    }

    printf("I2C bus recovery complete\n");
}

