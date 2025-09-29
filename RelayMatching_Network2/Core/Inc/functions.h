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
extern uint8_t Min_State , Max_State ;
extern float min_voltage;
extern float max_voltage;
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
}Relay_State_t;

typedef enum {
	RELAY_1,
	RELAY_2,
	RELAY_3
}Relay_ID_t;

// 3. PUBLIC FUNCTION PROTOTYPES
// ===================================================================
// These are the functions you will call from your main.c file.

/**
 * @brief CALIBRATION: Cycles through all relay combinations to find the ones that produce
 *        the minimum and maximum ADC readings. This MUST be run before other tests.
 * @retval HAL_StatusTypeDef HAL_OK on success, or HAL_ERROR on failure.
 */
HAL_StatusTypeDef Calibrate_And_Find_Ranges(void);

/**
 * @brief TEST 1: Measures the time it takes for the ADC value to settle after
 *        switching from the MIN state to the MAX state.
 * @note  Calibrate_And_Find_Ranges() must be called first.
 * @retval Settling time in milliseconds.
 */
uint32_t Measure_Settle_Time_Min_To_Max(void);

/**
 * @brief TEST 2: Measures the time it takes for the ADC value to settle after
 *        switching from the MAX state to the MIN state.
 * @note  Calibrate_And_Find_Ranges() must be called first.
 * @retval Settling time in milliseconds.
 */
uint32_t Measure_Settle_Time_Max_To_Min(void);

/**
 * @brief TEST 3 (SMOOTHNESS ANALOG): Measures the repeatability of the MIN state.
 *        It goes to MIN, then MAX, then back to MIN and measures the difference
 *        in the ADC reading. A low return value is good.
 * @note  Calibrate_And_Find_Ranges() must be called first.
 * @retval The absolute difference in ADC counts between two consecutive measurements of the MIN state.
 */
uint16_t Measure_State_Repeatability_Min(void);

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

/**
 * @brief TEST 4: Measures the normalized overshoot error when switching from the MIN to MAX state.
 * @return Normalized error (0.0 = perfect, >0 = overshoot). Negative on error.
 */
float Get_Relay_Min_to_Max_Overshoot_Error(void);

/**
 * @brief TEST 5: Measures the normalized undershoot error when switching from the MAX to MIN state.
 * @return Normalized error (0.0 = perfect, >0 = undershoot). Negative on error.
 */
float Get_Relay_Max_to_Min_Undershoot_Error(void);

#endif /* FUNCTIONS_H_ */
