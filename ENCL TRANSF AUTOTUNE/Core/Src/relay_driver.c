/*
 * relay_control.c
 *
 *  Created on: 20-Jan-2026
 *      Author: AdityaSingh
 */
#include <relay_driver.h>

/**
 * @brief Deactivates all three relays.
 */
void relay_reset(void) {
    HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY3_PIN, GPIO_PIN_RESET);
}

/**
 * @brief Sets the state of a single, specific relay.
 */
void relay_set_state(Relay_ID_t relayID, Relay_State_t state) {
    GPIO_PinState pin_state = (state == RELAY_ON) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    switch (relayID) {
        case RELAY_1: HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY1_PIN, pin_state); break;
        case RELAY_2: HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY2_PIN, pin_state); break;
        case RELAY_3: HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY3_PIN, pin_state); break;
        default: break;
    }
}

/**
 * @brief Sets relays to a specific combination based on a bitmask.
 * @param combination An 8-bit integer where bits 0, 1, and 2 correspond to relays 1, 2, and 3.
 */
void set_relay_combination(uint8_t combination)
{
    // Use the bits of 'combination' to set each relay state
    relay_set_state(RELAY_1, (combination & 0b001) ? RELAY_ON : RELAY_OFF);
    relay_set_state(RELAY_2, (combination & 0b010) ? RELAY_ON : RELAY_OFF);
    relay_set_state(RELAY_3, (combination & 0b100) ? RELAY_ON : RELAY_OFF);
}
