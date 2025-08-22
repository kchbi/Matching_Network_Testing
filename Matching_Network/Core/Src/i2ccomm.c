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





uint32_t Timeout = 100 ;

#define ConfigAddress 0x00
#define CalibrationAddress 0x05
#define VoltageAddress 0x02
#define CurrentAddress 0x04
#define ConfigSettings 0x4527
const float Maximum_Expected_Current = 1;
const float Shunt_Resistor_Value = 0.1f ;
const float Current_LSB = Maximum_Expected_Current/32768.0f;
const float calculated_cal_float = 0.00512f / (Current_LSB * Shunt_Resistor_Value);
const uint16_t calibration_value = (uint16_t)(calculated_cal_float + 0.5f);






HAL_StatusTypeDef I2C_Init(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
	if(HAL_I2C_IsDeviceReady(hi2c, ((DevAddress) << 1), 10,  Timeout) != HAL_OK)
	{
		printf("The I2C Device is not Ready \n");
		return HAL_ERROR;
	}
	printf("The Device is Ready \n");
	HAL_Delay(100);
	// --- THIS IS THE RESET CODE ---
	uint16_t reset_cmd = 0x8000;
	uint8_t tx_buffer[2];

	// Prepare the 2-byte command
	tx_buffer[0] = (reset_cmd >> 8);      // MSB = 0x80
	tx_buffer[1] = (reset_cmd & 0xFF);      // LSB = 0x00

	// Write the reset command to the Configuration Register (0x00)
	if(HAL_I2C_Mem_Write(hi2c, (DevAddress << 1), ConfigAddress, I2C_MEMADD_SIZE_8BIT, tx_buffer, 2, 100) != HAL_OK)
	{
	    printf("ERROR: Failed to send RESET command.\n");
	    return HAL_ERROR;
	}

	// Give the chip a moment to complete its internal reset
	HAL_Delay(5);
	printf("SUCCESS: Reset command sent. Continuing with configuration...\n");
	tx_buffer[0] = ConfigSettings >> 8 ;
	tx_buffer[1] = (ConfigSettings & 0xFF);
	if(HAL_I2C_Mem_Write(hi2c, ((DevAddress) << 1), ConfigAddress , I2C_MEMADD_SIZE_8BIT, tx_buffer, 2, Timeout) != HAL_OK)
	{
		printf("Failed to do the Configuration \n");
		return HAL_ERROR;
	}
	printf("Configuration was done Successfully \n");
	HAL_Delay(100);

	tx_buffer[0] = calibration_value >> 8 ;
	tx_buffer[1] = (calibration_value & 0xFF);

	if(HAL_I2C_Mem_Write(hi2c, ((DevAddress) << 1), CalibrationAddress , I2C_MEMADD_SIZE_8BIT, tx_buffer, 2, Timeout) != HAL_OK)
	{
		printf("Failed to do the Calibration \n");
		return HAL_ERROR;
	}
	printf("Calibration was done Successfully \n");
	HAL_Delay(100);
    // Read back configuration to verify
    uint8_t config_read[2];
    if(HAL_I2C_Mem_Read(hi2c, (DevAddress << 1), ConfigAddress,
                       I2C_MEMADD_SIZE_8BIT, config_read, 2, Timeout) == HAL_OK)
    {
        uint16_t actual_config = (config_read[0] << 8) | config_read[1];
        printf("Actual Config: 0x%04X\n", actual_config);

        if(actual_config != ConfigSettings) {
            printf("ERROR: Config write failed!\n");
        }
    }
	return HAL_OK;

}

HAL_StatusTypeDef I2C_ReadVoltage(I2C_HandleTypeDef *hi2c, uint16_t DevAddress , float *Voltage )
{
	uint8_t rx_buffer[2];
	if(HAL_I2C_Mem_Read(hi2c,((DevAddress<<1)), VoltageAddress ,  I2C_MEMADD_SIZE_8BIT, rx_buffer, 2, Timeout) != HAL_OK)
	{
		printf("Could not read the Voltage from Voltage Register \n");
		return HAL_ERROR;

	}

    // 2. Combine the two 8-bit bytes into one 16-bit unsigned integer (MSB first)
    uint16_t raw_voltage = (rx_buffer[0] << 8) | rx_buffer[1];
    printf("Raw Bus Voltage: 0x%04X\n", raw_voltage);

    // 3. Convert the raw value to Volts by multiplying by the LSB (1.25mV)
    //    The datasheet specifies a fixed LSB for voltage of 1.25 mV/bit.
    //    *voltage_V is used to "return" the float value via a pointer.
    *Voltage = raw_voltage * 0.00125f;

    // Optional: Print the result for debugging
    // printf("Raw Voltage: 0x%04X, Calculated Voltage: %.3f V\n", raw_voltage, *voltage_V);
    printf("Read the Voltage from the Voltage Register Successfully \n");

	return HAL_OK;


}

HAL_StatusTypeDef I2C_ReadCurrent(I2C_HandleTypeDef *hi2c, uint16_t DevAddress , float *Current )
{
	uint8_t rx_buffer[2];
	if(HAL_I2C_Mem_Read(hi2c,((DevAddress<<1)), CurrentAddress ,  I2C_MEMADD_SIZE_8BIT, rx_buffer, 2, Timeout) != HAL_OK)
	{
		printf("Could not read the Current from Current Register \n");
		return HAL_ERROR;

	}


    // 2. Combine the two 8-bit bytes into one 16-bit unsigned integer (MSB first)
    int16_t raw_current = (int16_t)((rx_buffer[0] << 8) | rx_buffer[1]);

    // 3. Convert the raw value to Ampere by multiplying by the LSB (1 mA)
    //    The datasheet specifies a fixed LSB for Current of 1 mA/bit.
    //    *Current is used to "return" the float value via a pointer.
    *Current = raw_current * Current_LSB;
    printf("Read the current from Current Register Successfully \n");

    // Optional: Print the result for debugging
    // printf("Raw Current: 0x%04X, Calculated Current: %.3f V\n", raw_current, *Current);

	return HAL_OK;


}
HAL_StatusTypeDef I2C_ReadShuntVoltage(I2C_HandleTypeDef *hi2c, uint16_t DevAddress , float *Voltage)
{
	uint8_t rx_buffer[2];
	if(HAL_I2C_Mem_Read(hi2c, (DevAddress << 1), 0x01, I2C_MEMADD_SIZE_8BIT, rx_buffer, 2, Timeout) != HAL_OK)
	{
		printf("Could not read Shunt Voltage\n");
		return HAL_ERROR;
	}
	int16_t raw = (int16_t)((rx_buffer[0] << 8) | rx_buffer[1]);
	printf("Raw Shunt Voltage: 0x%04X\n", raw);
	*Voltage = raw * 0.0000025f;  // 2.5 µV per bit
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
void I2C_ClearBus(I2C_HandleTypeDef *hi2c) {
    GPIO_InitTypeDef GPIO_InitStruct;
    __HAL_RCC_GPIOB_CLK_ENABLE();


    // 1. Release I2C peripheral
    HAL_I2C_DeInit(hi2c);

    // 2. Configure SCL & SDA as open-drain outputs
    GPIO_InitStruct.Pin = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;   // enable internal PU in case externals missing
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);

    // 3. Check if SDA is stuck low
    if (HAL_GPIO_ReadPin(I2C_GPIO_PORT, I2C_SDA_PIN) == GPIO_PIN_RESET) {
        printf("I2C SDA stuck low, pulsing SCL...\n");
        // Clock SCL up to 16 times
        for (int i = 0; i < 16; i++) {
            HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
            for (volatile int d=0; d<100; d++); // short delay
            HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
            for (volatile int d=0; d<100; d++);
        }
    }

    // 4. Generate STOP
    HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SDA_PIN, GPIO_PIN_RESET);
    for (volatile int d=0; d<100; d++);
    HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    for (volatile int d=0; d<100; d++);
    HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SDA_PIN, GPIO_PIN_SET);

    // 5. Re-init I2C peripheral
    if (HAL_I2C_Init(hi2c) != HAL_OK) {
        Error_Handler();
    }
    printf("I2C bus recovery complete.\n");
}

