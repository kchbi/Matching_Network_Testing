/*
 * functions.c
 *
 *  Created on: Jun 1, 2025
 *      Author: Aditya Singh
 */
#include "stm32f4xx_hal.h"     // Main HAL header (includes GPIO, TIM, etc.)
#include "main.h"
#include "functions.h"// Usually contains GPIO pin mappings and handles
#define MOTOR1_IN1 GPIO_PIN_7
#define MOTOR1_IN2 GPIO_PIN_8
#define MOTOR2_IN1 GPIO_PIN_9
#define MOTOR2_IN2 GPIO_PIN_10
uint32_t speed = 170;
uint32_t TIMEOUT_VALUE = 1000 ;
uint32_t Potentiometer1_data[1];
uint32_t Potentiometer2_data[1];

void Motor1DriveDirection(uint32_t speed)
{
    HAL_GPIO_WritePin(GPIOE, MOTOR1_IN1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, MOTOR1_IN2, GPIO_PIN_RESET);
    TIM1->CCR1 = speed;
}

void Motor1DriveReverseDirection(uint32_t speed)
{
    HAL_GPIO_WritePin(GPIOE, MOTOR1_IN1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, MOTOR1_IN2, GPIO_PIN_SET);
    TIM1->CCR1 = speed;
}

void Motor2DriveDirection(uint32_t speed) {
    HAL_GPIO_WritePin(GPIOE, MOTOR2_IN1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, MOTOR2_IN2, GPIO_PIN_RESET);
    TIM2->CCR1 = speed;
}

void Motor2DriveReverseDirection(uint32_t speed)
{
    HAL_GPIO_WritePin(GPIOE, MOTOR2_IN1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, MOTOR2_IN2, GPIO_PIN_SET);
    TIM2->CCR1 = speed;
}

int Lag(void)
{
    // Start the motor
    MotorMotor1DriveDirection1(170);
    uint32_t start_time_motor = HAL_GetTick();

    // Store initial potentiometer position
    uint32_t init_position = Potentiometer1_data[0];

    // Wait for a change in potentiometer reading
    while ((Potentiometer1_data[0] - init_position) < 5) {
        // Optionally add a timeout mechanism to avoid infinite loop
        if ((HAL_GetTick() - start_time_motor) > TIMEOUT_VALUE) {
            return -1; // Indicate timeout
        }
    }

    uint32_t start_time_pot = HAL_GetTick();
    uint32_t lagtime = start_time_pot - start_time_motor;
    return lagtime;
}

/**
* @brief  Gets adc data from ADS7828
* @param  address ADS7828 address set by A0 and A1 pins(default:00 = 0x48)
* @param  pinCfg configuration of SD and C[2..0] bits for
* 				input pin selection
* 				SD: 0 for differential, 1 for single ended
* 				C[2..0] input pin configuration
* 				by default, internal reference and ADC is turned on(bits PD[1..0])
* @param	adcdata pointer to target memory for adc data
* @retval HAL status
*/
HAL_StatusTypeDef ADS7828_readADC(uint8_t address, uint8_t pinCfg, uint16_t *adcdata)
{
	HAL_StatusTypeDef ret;
	uint8_t addata[2];

	// Send configuration register data
	ret = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(address<<1), &pinCfg, 1, 50);
	if(ret != HAL_OK)
	{
		return ret;
	}

	for(int i=0; i<5000; i++);

	// Receive voltage data, two bytes
	ret = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(address<<1)|0x01, addata, 2, 50);
	if(ret != HAL_OK)
	{
		return ret;
	}

	// Assemble adc reading data from two bytes
	*adcdata = ((addata[0] & 0x0F) << 8) | addata[1];
	return HAL_OK;
}

/**
* @brief  Get temperature from MAX30205
* @param  dev_address MAX30205 address set by A0,A1, A2 pins
* 				datasheet provides 8bit address, therefore no need
* 				to left shift in i2c functions (default=000 or 0x90 = (0x48 <<1))
* @param  config configuration register for MAX30205 (size 8b)
* 				configuration register address is 0x01
* @param	temp pointer to target memory for temperature data
* @retval HAL status
*/
HAL_StatusTypeDef MAX30205_readTemp(uint8_t dev_address, uint8_t config, float *temp)
{
	HAL_StatusTypeDef ret;
	uint8_t tempData[2];

	// Send configuration, reg 1
	ret = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(dev_address), 0x01, 1, &config, 1, 50);
	if(ret != HAL_OK)
	{
		return ret;
	}

	for(int i=0; i<5000; i++);

	// Get temperature data, reg 0, two bytes
	ret = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(dev_address) |0x01, 0x00, 1, tempData, 2, 50);
	if(ret != HAL_OK)
	{
		return ret;
	}

	// Convert to temperature
	// Datasheet shows that digits are powers of two in temperature degree C
	*temp = ( (tempData[0] << 8) | tempData[1]) *0.00390625;

	return HAL_OK;
}

/**
* @brief  Convert adc 16bit value to float voltage
* @param	vref reference voltage of adc(internal?)
* @param  adcval  value of adc to convert
* @param  voltage converted adc value
* @retval HAL status
*/
void adc_toVolt(float vref, uint8_t res, uint16_t adcval, float *voltage)
{
	*voltage = adcval * vref / (float)((1<<res) -1);
}



