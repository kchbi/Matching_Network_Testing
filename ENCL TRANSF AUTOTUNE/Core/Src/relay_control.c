/*
 * relay_control.c
 *
 *  Created on: 22-Jan-2026
 *      Author: AdityaSingh
 */
#include <relay_driver.h>


void TurnK1K2On(void)
{
	set_relay_combination(1);

}

void TurnK3On(void)
{
	set_relay_combination(2);
}

void TurnK4On(void)
{
	set_relay_combination(2);

}

void TurnRelayOff(void){
	relay_reset();
}
