/*
 * test.c
 *
 *  Created on: 12-Jan-2026
 *      Author: AdityaSingh
 */

#include "motor.h"
#include "sensor.h"
#include "main.h"

#define NUM_PARAMETERS 20
float parameters[NUM_PARAMETERS] = {0.0f};


void test_assembly(float *parameters){
	while (1) {
		motor_start_move(0,DIR_CW,1);
		if (sensor_state[SENSOR1] == true){
			while (1){
				motor_start_move(1,DIR_CW,1);


			}

		}


	}




}


uint8_t convert_Bool(bool *array){


}





void transmit_json(float *parameters){
    printf("DATA,");
    for (int i = 0; i < NUM_PARAMETERS; i++) {
        // We multiply by 1000.0 to send floats as integers with 3 decimal places. We will divide this by 1000 on the GUI Side to get the Real result
        printf("%ld", (int32_t)(parameters[i] * 1000.0f));
        if (i < NUM_PARAMETERS - 1) {
            printf(",");
        }
    }
    printf("\n");
    fflush(stdout);
}





