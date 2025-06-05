/*
 * functions.h
 *
 *  Created on: Jun 1, 2025
 *      Author: adity
 */

#ifndef INC_FUNCTIONS_H_
#define INC_FUNCTIONS_H_

extern uint32_t speed ;
extern uint32_t Potentiometer1_data[1];
extern uint32_t Potentiometer2_data[1];
void Motor1DriveDirection(uint32_t speed);
void Motor2DriveDirection(uint32_t speed);
void Motor1DriveReverseDirection(uint32_t speed);
void Motor2DriveReverseDirection(uint32_t speed);

#endif /* INC_FUNCTIONS_H_ */
