/*
 * uart_handler.h
 *
 *  Created on: Jun 22, 2025
 *      Author: adity
 */

#ifndef INC_UART_HANDLER_H_
#define INC_UART_HANDLER_H_


#include "main.h" // Includes HAL drivers and peripheral handles

/**
 * @brief Initializes the UART handler and starts listening for data.
 * @param huart Pointer to the UART_HandleTypeDef for the desired UART peripheral.
 */
void uart_handler_init(UART_HandleTypeDef *huart);

/**
 * @brief Checks if a complete command has been received.
 * @retval 1 if a command is ready, 0 otherwise.
 */
uint8_t uart_command_is_ready(void);

/**
 * @brief Gets a pointer to the received command buffer.
 * @retval A const pointer to the null-terminated command string.
 */
volatile uint8_t* uart_get_command_buffer(void);

/**
 * @brief Resets the command-ready flag and buffer for the next command.
 *        This should be called after a command has been processed.
 */
void uart_reset_for_next_command(void);


#endif /* INC_UART_HANDLER_H_ */
