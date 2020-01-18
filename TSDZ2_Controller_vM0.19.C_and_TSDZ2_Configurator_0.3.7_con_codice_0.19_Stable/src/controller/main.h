/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _MAIN_H_
#define _MAIN_H_

#include "config.h"

//=================================================================================================
// ENABLES
//=================================================================================================
#if ENABLE_VLCD6
#define ENABLE_VLCD6_COMPATIBILITY						1
#define ENABLE_VLCD5_COMPATIBILITY						0
#define ENABLE_XH18_COMPATIBILITY							0
#elif ENABLE_VLCD5
#define ENABLE_VLCD5_COMPATIBILITY						1
#define ENABLE_VLCD6_COMPATIBILITY						0
#define ENABLE_XH18_COMPATIBILITY							0
#elif ENABLE_XH18
#define ENABLE_XH18_COMPATIBILITY							1
#define ENABLE_VLCD6_COMPATIBILITY						0
#define ENABLE_VLCD5_COMPATIBILITY						0
#else
#define ENABLE_VLCD6_COMPATIBILITY						1
#define ENABLE_VLCD5_COMPATIBILITY						0
#define ENABLE_XH18_COMPATIBILITY							0
#endif 
//---------------------------------------------------------
#define ENABLE_DEBUG_UART											0
#define ENABLE_DEBUG_FIRMWARE									0
//---------------------------------------------------------
#define ENABLE_ASSIST_LEVEL_EMTB							1
//---------------------------------------------------------
#define CLR_SOFT_START_WHEN_CADENCE_RPM_ZERO	1
#if CLR_SOFT_START_WHEN_CADENCE_RPM_ZERO
#define CLR_SOFT_START_WHEN_SPEED_ZERO				0
#else
#define CLR_SOFT_START_WHEN_SPEED_ZERO				1
#endif
//---------------------------------------------------------
#define ENABLE_LOW_PASS_SOFT_START_RAMP				0
#if ENABLE_LOW_PASS_SOFT_START_RAMP
#define ENABLE_LINEAR_SOFT_START_RAMP					0
#else
#define ENABLE_LINEAR_SOFT_START_RAMP					1
#endif
//---------------------------------------------------------
#define ENABLE_BETA_CALC_HUMAN_FORCE					0
#define ENABLE_LAST_PEDAL_POWER_DIVISOR				1
//---------------------------------------------------------
#define ENABLE_PWM_ZERO_WHEN_ZERO_CADENCE_RPM	0
//---------------------------------------------------------
#define ENABLE_EEPROM_WRITE_IF_CHANGED				0
//---------------------------------------------------------
#if ENABLE_DISPLAY_ALWAYS_ON
#define ENABLE_DISPLAY_WORKING_FLAG						0
#endif
/**********************************************
VLCD6 Faults List:
----------------------------------------------
#define NO_FAULT															0
#define TEMPERATURE_PROTECTION								1
#define SHORT_CIRCUIT_PROTECTION							2
#define TURN_FAULT														3
#define MOTOR_PHASE_LOSS											4
#define TORQUE_FAULT													5
#define JAM_FAULT															6
#define EBIKE_WHEEL_BLOCKED										7
#define OVERVOLTAGE														8
***********************************************/
#define NO_FAULT															0
#define OVERTEMPERATURE												6 // E06
#if ENABLE_XH18_COMPATIBILITY
#define EBIKE_WHEEL_BLOCKED										4 // E04
#else
#define EBIKE_WHEEL_BLOCKED										7 // E07
#endif
#define OVERVOLTAGE														8 // E08
//---------------------------------------------------------
#define NO_FUNCTION														0
#define CLEAR_DISPLAY													0
#define FUNCTION_CODE_RANGE										9
#define PENDING_FUNCTION_ABORTED							14
#define DEFAULT_FUNCTION_ENABLED							15

#if ENABLE_XH18_COMPATIBILITY
#define DEFAULT_ENABLED_ON_OEM								2 // E02
#else
#define DEFAULT_ENABLED_ON_OEM								2 // E02
#endif
#define OFFROAD_FUNCTION_ENABLED							12
#define STREET_FUNCTION_ENABLED								13
#if ENABLE_XH18_COMPATIBILITY
#define STREET_ENABLED_ON_OEM									2 // E02
#define OFFROAD_ENABLED_ON_OEM								3 // E03
#else
#define STREET_ENABLED_ON_OEM									3 // E03
#define OFFROAD_ENABLED_ON_OEM								4 // E04
#endif
#define BOOST_FUNCTION_ENABLED								10
#define BOOST_FUNCTION_DISABLED								11
#if ENABLE_XH18_COMPATIBILITY
#define BOOST_DISABLED_ON_OEM									2 // E02
#define BOOST_ENABLED_ON_OEM									3 // E03
#else
#define BOOST_DISABLED_ON_OEM									1 // E01
#define BOOST_ENABLED_ON_OEM									5 // E05
#endif
#define EMTB_FUNCTION_ENABLED									16
#define EMTB_FUNCTION_DISABLED								17
#if ENABLE_XH18_COMPATIBILITY
#define EMTB_DISABLED_ON_OEM									2 // E02
#define EMTB_ENABLED_ON_OEM										3 // E03
#else
#define EMTB_DISABLED_ON_OEM									1 // E01
#define EMTB_ENABLED_ON_OEM										2 // E02
#endif

//=================================================================================================
// SYSTEM ERRORS
//=================================================================================================
#define NO_ERROR                                		0
#define ERROR_MOTOR_BLOCKED													1
#define ERROR_TORQUE_APPLIED_DURING_POWER_ON				2
#define ERROR_BRAKE_APPLIED_DURING_POWER_ON					3
#define ERROR_THROTTLE_APPLIED_DURING_POWER_ON			4
#define ERROR_NO_SPEED_SENSOR_DETECTED							5

//=================================================================================================
// CHECK SYSTEM
//=================================================================================================
#define MOTOR_BLOCKED_COUNTER_THRESHOLD             30		// 30  =>  3 seconds
#define MOTOR_BLOCKED_BATTERY_CURRENT_THRESHOLD_X5	8			// 8  =>  (8 * 0.826) / 5 = 1.3216 ampere  =>  (X) units = ((X * 0.826) / 5) ampere
#define MOTOR_BLOCKED_ERPS_THRESHOLD                10		// 10 ERPS
#define MOTOR_BLOCKED_RESET_COUNTER_THRESHOLD       100		// 100  =>  10 seconds

//=================================================================================================
// UART
//=================================================================================================
#define UART_RX_BUFFER_LEN   									7
#define RX_CHECK_CODE													(UART_RX_BUFFER_LEN - 1)															
#define UART_TX_BUFFER_LEN										9
#define TX_CHECK_CODE													(UART_TX_BUFFER_LEN - 1)
#define TX_STX																0x43
#define RX_STX																0x59															

//=================================================================================================
// EBIKE APP STATE MOTOR
//=================================================================================================
#define EBIKE_APP_STATE_MOTOR_COAST     			0
#define EBIKE_APP_STATE_MOTOR_STOP      			1
#define EBIKE_APP_STATE_MOTOR_STARTUP   			2
#define EBIKE_APP_STATE_MOTOR_COOL      			3
#define EBIKE_APP_STATE_MOTOR_RUNNING					4

//=================================================================================================
// PEDALING STATE
//=================================================================================================
#define STATE_NO_PEDALLING                		0
#define STATE_STARTUP_PEDALLING           		1
#define STATE_PEDALLING                   		2

//=================================================================================================
// BOOST STATE
//=================================================================================================
#define BOOST_STATE_BOOST_DISABLED        		0
#define BOOST_STATE_START_BOOST           		1
#define BOOST_STATE_BOOST                 		2
#define BOOST_STATE_END_BOOST             		3
#define BOOST_STATE_FADE                  		4
#define BOOST_STATE_BOOST_WAIT_TO_RESTART 		5

//=================================================================================================
// GENERAL CONFIG
//=================================================================================================
#define LIGHTS_ACTIVATED																		0	// lights activated (CONFIG_0 bit0)
#define WALK_ASSIST_ACTIVATED																0	// walk assist activated (CONFIG_0 bit1)
#define OFFROAD_MODE_ACTIVATED															0	// offroad mode activated (CONFIG_0 bit2)
#define EMTB_MODE_ACTIVATED																	0	// emtb mode activated (CONFIG_0 bit3)
#define CONFIG_0																						(uint8_t) ((LIGHTS_ACTIVATED)|(WALK_ASSIST_ACTIVATED << 1)|(OFFROAD_MODE_ACTIVATED << 2)|(EMTB_MODE_ACTIVATED << 3)|(ENABLE_EMTB_MODE_ON_STARTUP << 4))

#if MOTOR_TYPE_36V
#define MOTOR_TYPE	1
#elif MOTOR_TYPE_48V
#define MOTOR_TYPE	0
#else
#define MOTOR_TYPE	1
#endif
#define CONFIG_1																						(uint8_t) ((MOTOR_TYPE)|(EXPERIMENTAL_HIGH_CADENCE_MODE << 1)|(MOTOR_ASSISTANCE_WITHOUT_PEDAL_ROTATION << 2))

//=================================================================================================
// STREET CONFIG
//=================================================================================================
#define ENABLE_STREET_FEATURE																1 // enable street feature (but enable street mode by display)
#define STREET_CONFIG																				(uint8_t) ((ENABLE_STREET_FEATURE)|(ENABLE_STREET_MODE_ON_STARTUP << 1)|(ENABLE_STREET_POWER_LIMIT << 2))
#define STREET_POWER_LIMIT_DIV25         										(uint8_t) (STREET_POWER_LIMIT / 25)

//=================================================================================================
// BATTERY CONFIG
//=================================================================================================
// This is the current that motor will draw from the battery
// Higher value will give higher torque and the limit of the controller is 16 amps
#define ADC_BATTERY_CURRENT_MAX															(uint8_t) (ADC_BATTERY_CURRENT_MAX_LIMIT / 0.625)

#define BATTERY_MAX_CURRENT																	(uint8_t) (BATTERY_MAX_CURRENT_FLOAT)

// Battery low voltage cut off
#define BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0										(uint8_t) ((uint16_t)(BATTERY_LOW_VOLTAGE_CUT_OFF_DIV10 * 10) & 0x00FF)
#define BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1										(uint8_t) (((uint16_t)(BATTERY_LOW_VOLTAGE_CUT_OFF_DIV10 * 10) >> 8) & 0x00FF)

// Battery pack resistance
#define BATTERY_PACK_RESISTANCE_0														(uint8_t) (BATTERY_PACK_RESISTANCE & 0x00FF)
#define BATTERY_PACK_RESISTANCE_1														(uint8_t) ((BATTERY_PACK_RESISTANCE >> 8) & 0x00FF)

// Max battery power
#define TARGET_MAX_BATTERY_POWER_DIV25      								(uint8_t) (TARGET_MAX_BATTERY_POWER / 25)

// ADC Battery voltage
#define ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X512					(uint16_t) (DIVISOR_FOR_CUTOFF_CALC)
#define ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X256 				(ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X512 >> 1)
#define ADC8BITS_BATTERY_VOLTAGE_PER_ADC_STEP_INVERSE_X256	(ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X256 << 2)

// ADC Battery current
// 1A per 5 steps of ADC_10bits
#define ADC_BATTERY_CURRENT_PER_ADC_STEP_X512								102

// ADC Battery SOC voltage per ADC step (x10000)
#define SOC_ADC_BATTERY_VOLTAGE_PER_ADC_STEP_X10000 				(uint16_t) (SOC_ADC_BATTERY_VOLTAGE_PER_ADC_STEP * 10000)	// Samsung INR18650-25R cells

#if ENABLE_VLCD6_COMPATIBILITY || ENABLE_XH18_COMPATIBILITY
#define LI_ION_CELL_VOLTS_5																	(float)LI_ION_CELL_OVERVOLT
#define LI_ION_CELL_VOLTS_4																	(float)LI_ION_CELL_VOLTS_100
#define LI_ION_CELL_VOLTS_3																	(float)LI_ION_CELL_VOLTS_75
#define LI_ION_CELL_VOLTS_2																	(float)LI_ION_CELL_VOLTS_50
#define LI_ION_CELL_VOLTS_1																	(float)LI_ION_CELL_VOLTS_25
#define LI_ION_CELL_VOLTS_0																	(float)LI_ION_CELL_EMPTY
#endif

#if ENABLE_VLCD5_COMPATIBILITY
#if ENABLE_VLCD5_BATTERY_SOC_4_BARS
#define LI_ION_CELL_VOLTS_5																	(float)LI_ION_CELL_OVERVOLT
#define LI_ION_CELL_VOLTS_4																	(float)LI_ION_CELL_VOLTS_100
#define LI_ION_CELL_VOLTS_3																	(float)LI_ION_CELL_VOLTS_75
#define LI_ION_CELL_VOLTS_2																	(float)LI_ION_CELL_VOLTS_50
#define LI_ION_CELL_VOLTS_1																	(float)LI_ION_CELL_VOLTS_25
#define LI_ION_CELL_VOLTS_0																	(float)LI_ION_CELL_EMPTY
#else
#define LI_ION_CELL_VOLTS_7																	(float)LI_ION_CELL_OVERVOLT
#define LI_ION_CELL_VOLTS_6																	(float)LI_ION_CELL_VOLTS_100
#define LI_ION_CELL_VOLTS_5																	(float)((((LI_ION_CELL_VOLTS_100 - LI_ION_CELL_VOLTS_25) / 5) * 4) + LI_ION_CELL_VOLTS_25)
#define LI_ION_CELL_VOLTS_4																	(float)((((LI_ION_CELL_VOLTS_100 - LI_ION_CELL_VOLTS_25) / 5) * 3) + LI_ION_CELL_VOLTS_25)
#define LI_ION_CELL_VOLTS_3																	(float)((((LI_ION_CELL_VOLTS_100 - LI_ION_CELL_VOLTS_25) / 5) * 2) + LI_ION_CELL_VOLTS_25)
#define LI_ION_CELL_VOLTS_2																	(float)((((LI_ION_CELL_VOLTS_100 - LI_ION_CELL_VOLTS_25) / 5) * 1) + LI_ION_CELL_VOLTS_25)
#define LI_ION_CELL_VOLTS_1																	(float)LI_ION_CELL_VOLTS_25
#define LI_ION_CELL_VOLTS_0																	(float)LI_ION_CELL_EMPTY
#endif
#endif

//=================================================================================================
// WHEEL CONFIG
//=================================================================================================
#define WHEEL_PERIMETER_0																		(uint8_t) (WHEEL_PERIMETER & 0x00FF)
#define WHEEL_PERIMETER_1																		(uint8_t) ((WHEEL_PERIMETER >> 8) & 0x00FF)
#define OEM_WHEEL_SPEED_DIVISOR_0														(uint8_t) (OEM_WHEEL_SPEED_DIVISOR & 0x00FF)
#define OEM_WHEEL_SPEED_DIVISOR_1														(uint8_t) ((OEM_WHEEL_SPEED_DIVISOR >> 8) & 0x00FF)

//=================================================================================================
// ASSIST PEDAL LEVELS
//=================================================================================================
#define ECO																									0
#define TOUR																								1
#define SPORT																								2
#define TURBO																								3
#define EMTB																								4

//=================================================================================================
// ASSIST PEDAL LEVELS INDEX
//=================================================================================================
#define ASSIST_PEDAL_LEVEL0																	0x10
#define ASSIST_PEDAL_LEVEL1																	0x40
#define ASSIST_PEDAL_LEVEL2																	0x02
#define ASSIST_PEDAL_LEVEL3																	0x04
#define ASSIST_PEDAL_LEVEL4																	0x08

//=================================================================================================
// SOFT START RAMP
//=================================================================================================
#define THRESHOLD_SOFT_START_PAS_CADENCE										(uint8_t) 1
#define INITIAL_SOFT_START_ASSIST_VALUE											(uint8_t) (INITIAL_SOFT_START_ASSIST_VALUE_X10 / 10)

//=================================================================================================
// EMTB MODE
//=================================================================================================
#define EMTB_MIN_CADENCE																		(uint8_t) 80
#define EMTB_MAX_CADENCE																		(uint8_t) 200
#define EMTB_START_ASSIST_LEVEL_FACTOR											(uint8_t) (EMTB_START_ASSIST_LEVEL_FACTOR_X10 / 10)

//=================================================================================================
// ASSIST LEVELS CONFIG
//=================================================================================================
#define ASSIST_LEVEL_FACTOR_1																(uint8_t) (ASSIST_LEVEL_FACTOR_X10_1 / 10)
#define ASSIST_LEVEL_FACTOR_2																(uint8_t) (ASSIST_LEVEL_FACTOR_X10_2 / 10)
#define ASSIST_LEVEL_FACTOR_3																(uint8_t) (ASSIST_LEVEL_FACTOR_X10_3 / 10)
#define ASSIST_LEVEL_FACTOR_4																(uint8_t) (ASSIST_LEVEL_FACTOR_X10_4 / 10)
#define ASSIST_LEVEL_FACTOR_X10															ASSIST_LEVEL_FACTOR_2

//=================================================================================================
// BOOST ASSIST CONFIG
//=================================================================================================
#define STARTUP_MOTOR_POWER_BOOST_FEATURE_ENABLED   				0	// startup power boost can be enabled only by display
#define STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_1						(uint8_t) ((STARTUP_MOTOR_BOOST_ASSIST_LEVEL_PERCENT_1 * 28) / 100)
#define STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_2						(uint8_t) ((STARTUP_MOTOR_BOOST_ASSIST_LEVEL_PERCENT_2 * 28) / 100)
#define STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_3						(uint8_t) ((STARTUP_MOTOR_BOOST_ASSIST_LEVEL_PERCENT_3 * 28) / 100)
#define STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_4  					(uint8_t) ((STARTUP_MOTOR_BOOST_ASSIST_LEVEL_PERCENT_4 * 28) / 100)
#define STARTUP_MOTOR_POWER_BOOST_TIME											(uint8_t) (STARTUP_MOTOR_POWER_BOOST_TIME_DIV10 * 10)
#define STARTUP_MOTOR_POWER_BOOST_FADE_TIME									(uint8_t) (STARTUP_MOTOR_POWER_BOOST_FADE_TIME_DIV10 * 10)
#if STARTUP_BOOST_WHEN_SPEED_IS_ZERO
#define STARTUP_MOTOR_POWER_BOOST_STATE	0
#elif STARTUP_BOOST_WHEN_CADENCE_IS_ZERO
#define STARTUP_MOTOR_POWER_BOOST_STATE	1
#else
#define STARTUP_MOTOR_POWER_BOOST_STATE	0
#endif

//=================================================================================================
// WALK ASSIST LEVELS CONFIG
//=================================================================================================
#define WALK_ASSIST_PWM_DUTY_CYCLE_LEVEL_0									(uint8_t) ((WALK_ASSIST_PWM_LEVEL_0 * 255) / 100)
#define WALK_ASSIST_PWM_DUTY_CYCLE_LEVEL_1 									(uint8_t) ((WALK_ASSIST_PWM_LEVEL_1 * 255) / 100)
#define WALK_ASSIST_PWM_DUTY_CYCLE_LEVEL_2									(uint8_t) ((WALK_ASSIST_PWM_LEVEL_2 * 255) / 100)
#define WALK_ASSIST_PWM_DUTY_CYCLE_LEVEL_3									(uint8_t) ((WALK_ASSIST_PWM_LEVEL_3 * 255) / 100)
#define WALK_ASSIST_PWM_DUTY_CYCLE_LEVEL_4									(uint8_t) ((WALK_ASSIST_PWM_LEVEL_4 * 255) / 100)
#define WALK_ASSIST_MAX_RAMP_TIME														(uint8_t) (WALK_ASSIST_MAX_RAMP_TIME_DIV10 * 10)
#define WALK_ASSIST_OFF_DELAY_PWM														(uint8_t) ((WALK_ASSIST_OFF_DELAY_PWM_DIV10 * 255) / 100)
#define WALK_ASSIST_OFF_DELAY_TIME_0												(uint8_t) ((uint16_t)(WALK_ASSIST_OFF_DELAY_TIME_DIV10 * 10) & 0x00FF)
#define WALK_ASSIST_OFF_DELAY_TIME_1												(uint8_t) (((uint16_t)(WALK_ASSIST_OFF_DELAY_TIME_DIV10 * 10) >> 8) & 0x00FF)
#define WALK_ASSIST_MIN_VALUE																(uint8_t) 0
#define WALK_ASSIST_MAX_VALUE																(uint8_t) 100

//=================================================================================================
// MOTOR CONFIG
//=================================================================================================
#define MOTOR_MAX_POWER																			250
#define MOTOR_MAX_POWER_DIV10																(uint8_t) (MOTOR_MAX_POWER / 10)

//=================================================================================================
// MOTOR PWM DUTY CYCLE
//=================================================================================================
// 5 erps minimum speed; 1/5 = 200ms; 200ms/64us = 3125
#define PWM_CYCLES_COUNTER_MAX															(uint16_t) (PWM_CYCLES_SECOND / 5)

// Target: ramp 5 amps per second
// every second has 15625 PWM cycles interrupts
// 1 ADC battery current step --> 0.625 amps
// 5 / 0.625 = 8 (we need to do 8 steps ramp up per second)
// 15625 / 8 = 1953
#define ADC_BATTERY_CURRENT_RAMP_UP_INVERSE_STEP						(uint16_t) (PWM_CYCLES_SECOND / (CURRENT_RAMP / 0.625))

// Middle PWM duty cycle max
#define MIDDLE_PWM_DUTY_CYCLE_MAX														(uint8_t) (PWM_DUTY_CYCLE_MAX / 2)

// Duty cycle ramp (up/down)
#define PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP									(uint16_t) ((PWM_DUTY_CYCLE_RAMP_UP_DIV1000 * 1000) / (1000000 / PWM_CYCLES_SECOND))
#define PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP								(uint16_t) ((PWM_DUTY_CYCLE_RAMP_DOWN_DIV1000 * 1000) / (1000000 / PWM_CYCLES_SECOND))

//=================================================================================================
// MOTOR ROTOR CONFIG
//=================================================================================================
// 0.625 amps each unit
#define ADC_MOTOR_PHASE_CURRENT_MAX													(uint8_t) (MOTOR_PHASE_CURRENT_MAX_AMP / 0.625)

#define MOTOR_ROTOR_ANGLE_90    														(63  + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_150   														(106 + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_210   														(148 + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_270   														(191 + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_330   														(233 + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_30    														(20  + MOTOR_ROTOR_OFFSET_ANGLE)

//=================================================================================================
// MOTOR STATES
//=================================================================================================
#define BLOCK_COMMUTATION 			                						1
#define SINEWAVE_INTERPOLATION_60_DEGREES 	    						2

#define MOTOR_CONTROLLER_STATE_OK			          						1
#define MOTOR_CONTROLLER_STATE_BRAKE			      						2
#define MOTOR_CONTROLLER_STATE_OVER_CURRENT		  						4
#define MOTOR_CONTROLLER_STATE_UNDER_VOLTAGE								8
#define MOTOR_CONTROLLER_STATE_THROTTLE_ERROR								16
#define MOTOR_CONTROLLER_STATE_MOTOR_BLOCKED								32

//=================================================================================================
// PAS
//=================================================================================================
// x = (1/(150RPM/60)) / (0.000064)
// PAS_ABSOLUTE_MAX_CADENCE_PWM_CYCLE_TICKS = (x / PAS_NUMBER_MAGNETS)
#define PAS_ABSOLUTE_MAX_CADENCE_PWM_CYCLE_TICKS  					(uint16_t) (6250 / PAS_NUMBER_MAGNETS)	// max hard limit to 150 RPM PAS cadence
#define MIN_PAS_CADENCE_RPM																	10
#if(MIN_PAS_CADENCE_RPM == 10)
#define PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS  					(uint16_t) (93750 / PAS_NUMBER_MAGNETS)	// min hard limit to 10 RPM PAS cadence
#elif(MIN_PAS_CADENCE_RPM == 5)
#define PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS  					(uint16_t) (187500 / PAS_NUMBER_MAGNETS)	// min hard limit to 5 RPM PAS cadence
#else
#define PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS  					(uint16_t) (375000 / PAS_NUMBER_MAGNETS)	// min hard limit to 2.5 RPM PAS cadence
#endif
#define PAS_NUMBER_MAGNETS_X2 															(uint8_t) (PAS_NUMBER_MAGNETS * 2)
#define THRESHOLD_PAS_BACKWARDS_CADENCE_RPM									(uint8_t) 40

//=================================================================================================
// TORQUE SENSOR
//=================================================================================================
#define ADC_STEP_PEDAL_TORQUE_X100                         	(uint16_t) (PEDAL_TORQUE_SENSOR_UNIT * 100)
#define TORQUE_SENSOR_THRESHOLD_HI													(uint8_t) 12
#define TORQUE_SENSOR_THRESHOLD_LOW													(uint8_t) 12

#if ENABLE_LAST_PEDAL_POWER_DIVISOR
#define PEDAL_POWER_DIVISOR																	(uint32_t) (96 / AVERAGE_TORQUE_FACTOR)
#else
#define PEDAL_POWER_DIVISOR																	(uint32_t) (105 / AVERAGE_TORQUE_FACTOR)
#endif
//=================================================================================================
// LIGHTS TIMING
//=================================================================================================
#define DELAY_LIGHTS_ON																			50 // x0.1 seconds

//=================================================================================================
// EEPROM ADDRESS MAP
//=================================================================================================
#define KEY																									(uint8_t)(MAGIC_BYTE)
#define KEY2																								(uint8_t)(~MAGIC_BYTE)
#define EEPROM_BASE_ADDR																		0x4000
#define ADDR_KEY                                 						EEPROM_BASE_ADDR
#define ADDR_ASSIST_LEVEL_FACTOR_X10             						1 + EEPROM_BASE_ADDR
#define ADDR_CONFIG_0                            						2 + EEPROM_BASE_ADDR
#define ADDR_BATTERY_MAX_CURRENT                 						3 + EEPROM_BASE_ADDR
#define ADDR_MOTOR_POWER_X10                     						4 + EEPROM_BASE_ADDR
#define ADDR_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0   						5 + EEPROM_BASE_ADDR
#define ADDR_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1   						6 + EEPROM_BASE_ADDR
#define ADDR_WHEEL_PERIMETER_0                   						7 + EEPROM_BASE_ADDR
#define ADDR_WHEEL_PERIMETER_1                   						8 + EEPROM_BASE_ADDR
#define ADDR_WHEEL_MAX_SPEED                     						9 + EEPROM_BASE_ADDR
#define ADDR_CONFIG_1                            						10 + EEPROM_BASE_ADDR
#define ADDR_STREET_CONFIG                      						11 + EEPROM_BASE_ADDR
#define ADDR_STREET_SPEED_LIMIT                 						12 + EEPROM_BASE_ADDR
#define ADDR_STREET_POWER_LIMIT_DIV25           						13 + EEPROM_BASE_ADDR
#define ADDR_BATTERY_CELLS_NUMBER														14 + EEPROM_BASE_ADDR
#define ADDR_BATTERY_PACK_RESISTANCE_0											15 + EEPROM_BASE_ADDR
#define ADDR_BATTERY_PACK_RESISTANCE_1											16 + EEPROM_BASE_ADDR
#define ADDR_OEM_WHEEL_SPEED_DIVISOR_0											17 + EEPROM_BASE_ADDR
#define ADDR_OEM_WHEEL_SPEED_DIVISOR_1											18 + EEPROM_BASE_ADDR
#define ADDR_ASSIST_LEVEL_FACTOR_1													19 + EEPROM_BASE_ADDR
#define ADDR_ASSIST_LEVEL_FACTOR_2													20 + EEPROM_BASE_ADDR
#define ADDR_ASSIST_LEVEL_FACTOR_3													21 + EEPROM_BASE_ADDR
#define ADDR_ASSIST_LEVEL_FACTOR_4													22 + EEPROM_BASE_ADDR
#define ADDR_STARTUP_MOTOR_POWER_BOOST_STATE								23 + EEPROM_BASE_ADDR
#define ADDR_STARTUP_MOTOR_POWER_BOOST_FEATURE_ENABLED			24 + EEPROM_BASE_ADDR
#define ADDR_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_1				25 + EEPROM_BASE_ADDR
#define ADDR_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_2				26 + EEPROM_BASE_ADDR
#define ADDR_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_3				27 + EEPROM_BASE_ADDR
#define ADDR_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_4    		28 + EEPROM_BASE_ADDR
#define ADDR_STARTUP_MOTOR_POWER_BOOST_TIME									29 + EEPROM_BASE_ADDR
#define ADDR_STARTUP_MOTOR_POWER_BOOST_FADE_TIME						30 + EEPROM_BASE_ADDR
#define ADDR_STARTUP_MOTOR_POWER_BOOST_LIMIT_MAX_POWER			31 + EEPROM_BASE_ADDR
#define ADDR_TARGET_MAX_BATTERY_POWER_DIV25									32 + EEPROM_BASE_ADDR
#define ADDR_TEMPERATURE_LIMIT_FEATURE_ENABLED							33 + EEPROM_BASE_ADDR
#define ADDR_MOTOR_TEMPERATURE_MIN_VALUE_LIMIT							34 + EEPROM_BASE_ADDR
#define ADDR_MOTOR_TEMPERATURE_MAX_VALUE_LIMIT							35 + EEPROM_BASE_ADDR
#define ADDR_WALK_ASSIST_PERCENTAGE_CURRENT									36 + EEPROM_BASE_ADDR
#define ADDR_WALK_ASSIST_PWM_DUTY_CYCLE_LEVEL_0							37 + EEPROM_BASE_ADDR
#define ADDR_WALK_ASSIST_PWM_DUTY_CYCLE_LEVEL_1							38 + EEPROM_BASE_ADDR
#define ADDR_WALK_ASSIST_PWM_DUTY_CYCLE_LEVEL_2							39 + EEPROM_BASE_ADDR
#define ADDR_WALK_ASSIST_PWM_DUTY_CYCLE_LEVEL_3							40 + EEPROM_BASE_ADDR
#define ADDR_WALK_ASSIST_PWM_DUTY_CYCLE_LEVEL_4							41 + EEPROM_BASE_ADDR
#define ADDR_WALK_ASSIST_MAX_RAMP_TIME											42 + EEPROM_BASE_ADDR
#define ADDR_WALK_ASSIST_OFF_DELAY_PWM											43 + EEPROM_BASE_ADDR
#define ADDR_WALK_ASSIST_OFF_DELAY_TIME_0										44 + EEPROM_BASE_ADDR
#define ADDR_WALK_ASSIST_OFF_DELAY_TIME_1										45 + EEPROM_BASE_ADDR
#define ADDR_KEY2																						46 + EEPROM_BASE_ADDR
#define EEPROM_BYTES_STORED																	47

#endif // _MAIN_H_
