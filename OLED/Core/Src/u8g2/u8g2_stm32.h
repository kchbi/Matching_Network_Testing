/*
 * u8g2_stm32.h
 *
 *  Created on: Jun 18, 2025
 *      Author: adity
 */

#ifndef SRC_U8G2_U8G2_STM32_H_
#define SRC_U8G2_U8G2_STM32_H_


#include "stm32f4xx_hal.h" // CHANGE THIS TO YOUR MCU's HAL HEADER (e.g., stm32g0xx_hal.h)
#include "u8g2.h"

// The I2C address from your board is 0x78
#define OLED_I2C_ADDR   0x78

// u8g2 requires a callback function for I2C communication
uint8_t u8x8_byte_i2c_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
// u8g2 requires a callback function for delays
uint8_t u8g2_gpio_and_delay_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

#endif /* SRC_U8G2_U8G2_STM32_H_ */
