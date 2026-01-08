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
extern float POS_MIN_TUNE;
extern float POS_MAX_TUNE;

extern float POS_MIN_LOAD;
extern float POS_MAX_LOAD;

extern float POS_HOME_TUNE;
extern float POS_HOME_LOAD;

#define MOVE_TOLERANCE_ADC  (20)       // <<< CHANGED: Tolerance for reaching a position
// ===================================================================
// 2. EXTERN VARIABLE DECLARATIONS
// ===================================================================
/**
 * @brief Global variable to hold the DMA results for the 2 ADC channels.
 *        Defined in main.c, declared here to be visible across files.
 */
extern uint16_t volatile ADC_IN[9];
extern float Max_PCurrent_Tune, Max_NCurrent_Tune, Voltage_Tune ;
extern float Max_PCurrent_Load, Max_NCurrent_Load, Voltage_Load ;

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
    MOTOR_STOP,
    MOTOR_FORWARD,
    MOTOR_REVERSE
} MotorDirection_t;


typedef enum{
	ON,
	OFF
}DIO_t;
void ModeControlADIOBoard(DIO_t STATE  );
void relay_reset(void);
void set_relay_combination(uint8_t combination);
void relay_set_state(uint16_t GPIO_Pin, DIO_t state) ;

typedef enum {
    PARAM_X1_TIME_MIN_MAX ,
    PARAM_X1_TIME_MAX_MIN,
    PARAM_X2_TIME_MIN_MAX ,
    PARAM_X2_TIME_MAX_MIN ,
    PARAM_X1_P15V_I_MIN_MAX ,
	PARAM_X1_N15V_I_MIN_MAX ,
	PARAM_X1_24V_I_MIN_MAX ,
    PARAM_X1_P15V_I_MAX_MIN ,
	PARAM_X1_N15V_I_MAX_MIN ,
	PARAM_X1_24V_I_MAX_MIN ,
    PARAM_X2_P15V_I_MIN_MAX ,
	PARAM_X2_N15V_I_MIN_MAX ,
	PARAM_X2_24V_I_MIN_MAX ,
    PARAM_X2_P15V_I_MAX_MIN ,
	PARAM_X2_N15V_I_MAX_MIN ,
	PARAM_X2_24V_I_MAX_MIN ,
	PARAM_X1_MIN_POS_V ,
	PARAM_X1_MAX_POS_V ,
	PARAM_X2_MIN_POS_V ,
	PARAM_X2_MAX_POS_V ,
	PARAM_X1_STEP_MIN_MAX,
	PARAM_X1_STEP_MAX_MIN,
	PARAM_X2_STEP_MIN_MAX,
	PARAM_X2_STEP_MAX_MIN
} ParameterIndex_t;


typedef enum{

	PARAM_M1_ADC_INPUT_M,
	PARAM_M2_ADC_INPUT_M,
	PARAM_SINE_ADC_INPUT,
	PARAM_FEEDBACK_MANUALORAUTOMATIC,
	PARAM_PO_LOAD,
	PARAM_PO_TUNE,
	PARAM_RF_MATCH_PHASE,
	RF_POWER,
	RF_LINE_IMPEDANCE
}ADCIndex_t;
// ===================================================================
// 4. PUBLIC FUNCTION PROTOTYPES
// ===================================================================
uint32_t ADC_MIN_TO_ADC_MAX_M1(void);
uint32_t ADC_MAX_TO_ADC_MIN_M1(void);
uint32_t ADC_MIN_TO_ADC_MAX_M2(void);
uint32_t ADC_MAX_TO_ADC_MIN_M2(void);

float Get_M1_Min_to_Max_Smoothness(void);
float Get_M1_Max_to_Min_Smoothness(void);
float Get_M2_Min_to_Max_Smoothness(void);
float Get_M2_Max_to_Min_Smoothness(void);
void motor1_set_state(float speed);
void motor2_set_state(float speed);
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
