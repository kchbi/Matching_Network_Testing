/*
 * functions.h
 *
 *  Created on: Jun 1, 2025
 *      Author: Aditya Singh
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

// This is an "include guard". It prevents the file from being included
// multiple times, which would cause compilation errors.

// ===================================================================
// 1. PUBLIC CONSTANTS
// ===================================================================
extern uint16_t ADC_POS_MIN_M1;
extern uint16_t ADC_POS_MAX_M1;

extern uint16_t ADC_POS_MIN_M2;
extern uint16_t ADC_POS_MAX_M2;

extern uint16_t ADC_POS_HOME_M1;
extern uint16_t ADC_POS_HOME_M1;

#define MOVE_TOLERANCE_ADC  (20)       // <<< CHANGED: Tolerance for reaching a position
// ===================================================================
// 2. EXTERN VARIABLE DECLARATIONS
// ===================================================================
/**
 * @brief Global variable to hold the DMA results for the 2 ADC channels.
 *        Defined in main.c, declared here to be visible across files.
 */
extern uint16_t volatile Pot1_2[2];
extern float Max_PCurrent_M1, Max_NCurrent_M1, Voltage_M1 , Max_PoCurrent_M1;
extern float Max_PCurrent_M2, Max_NCurrent_M2, Voltage_M2 , Max_PoCurrent_M2;

/**
 * @brief Global variable to hold the result of the Test
 *        Defined in main.c, declared here to be visible across files.
 */

#define NUM_PARAMETERS 24
extern float parameters[NUM_PARAMETERS];

// ===================================================================
// 3. PUBLIC DATA TYPES
// ===================================================================
typedef enum {
    RELAY_ON,
    RELAY_OFF
} Relay_State_t;

typedef enum {
	RELAY_1,
	RELAY_2,
	RELAY_3
}Relay_ID_t;


typedef enum {
    PARAM_X1_TIME_MIN_MAX = 0,
    PARAM_X1_TIME_MAX_MIN = 1,
    PARAM_X2_TIME_MIN_MAX = 2,
    PARAM_X2_TIME_MAX_MIN = 3,
    PARAM_X1_P15V_I_MIN_MAX = 4,
	PARAM_X1_N15V_I_MIN_MAX = 5,
	PARAM_X1_24V_I_MIN_MAX = 6,
    PARAM_X1_P15V_I_MAX_MIN = 7,
	PARAM_X1_N15V_I_MAX_MIN = 8,
	PARAM_X1_24V_I_MAX_MIN = 9,
    PARAM_X2_P15V_I_MIN_MAX = 10,
	PARAM_X2_N15V_I_MIN_MAX = 11,
	PARAM_X2_24V_I_MIN_MAX = 12,
    PARAM_X2_P15V_I_MAX_MIN = 13,
	PARAM_X2_N15V_I_MAX_MIN = 14,
	PARAM_X2_24V_I_MAX_MIN = 15,
	PARAM_X1_MIN_POS_V = 16,
	PARAM_X1_MAX_POS_V = 17,
	PARAM_X2_MIN_POS_V = 18,
	PARAM_X2_MAX_POS_V = 19,
	PARAM_X1_STEP_MIN_MAX=20,
	PARAM_X1_STEP_MAX_MIN=21,
	PARAM_X2_STEP_MIN_MAX = 22,
	PARAM_X2_STEP_MAX_MIN = 23
} ParameterIndex_t;

// ===================================================================
// 4. PUBLIC FUNCTION PROTOTYPES
// ===================================================================
uint32_t ADC_MIN_TO_ADC_MAX_M1(void);
uint32_t ADC_MAX_TO_ADC_MIN_M1(void);
uint32_t ADC_MIN_TO_ADC_MAX_M2(void);
uint32_t ADC_MAX_TO_ADC_MIN_M2(void);

void relay_set_state(Relay_ID_t relayID, Relay_State_t state);

float Get_M1_Min_to_Max_Smoothness(void);
float Get_M1_Max_to_Min_Smoothness(void);
float Get_M2_Min_to_Max_Smoothness(void);
float Get_M2_Max_to_Min_Smoothness(void);
void motor1_set_state(MotorDirection_t direction, uint32_t speed);
void motor2_set_state(MotorDirection_t direction, uint32_t speed);
float FindMaxPositionVoltageMotor1();
float FindMinPositionVoltageMotor1();
float FindMaxPositionVoltageMotor2();
float FindMinPositionVoltageMotor2();
void moveMotor1ToADCValue(uint16_t targetADC, uint16_t tolerance);
void moveMotor1ToADCValue(uint16_t targetADC, uint16_t tolerance);
void moveMotor2ToADCValue(uint16_t targetADC, uint16_t tolerance);
void moveMotor2ToADCValue(uint16_t targetADC, uint16_t tolerance);
void moveMotor1ToADCValue_And_Measure_Smoothness(uint16_t targetADC, uint32_t expected_duration_ms, uint16_t tolerance, int32_t* max_error_out);
void moveMotor2ToADCValue_And_Measure_Smoothness(uint16_t targetADC, uint32_t expected_duration_ms, uint16_t tolerance, int32_t* max_error_out);
void Update_Home_Positions(void);

#endif /* FUNCTIONS_H_ */
