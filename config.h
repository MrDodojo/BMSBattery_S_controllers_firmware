/*
 * config.h
 *
 *  Automatically created by OSEC Parameter Configurator
 *  Author: stancecoke
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define NUMBER_OF_PAS_MAGS 10
#define limit 28
#define timeout 3125
#define wheel_circumference 2230L
#define limit_without_pas 25
#define ADC_THROTTLE_MIN_VALUE 43
#define ADC_THROTTLE_MAX_VALUE 183
#define BATTERY_VOLTAGE_MIN_VALUE 80L
#define BATTERY_CURRENT_MAX_VALUE 45L
#define PHASE_CURRENT_MAX_VALUE 135L
#define REGEN_CURRENT_MAX_VALUE 23L
#define MOTOR_ROTOR_DELTA_PHASE_ANGLE_RIGHT 234
#define current_cal_a 45
#define LEVEL_1 12
#define LEVEL_2 21
#define LEVEL_3 30
#define LEVEL_4 59
#define LEVEL_5 100
#define MORSE_TIME_1 50
#define MORSE_TIME_2 50
#define MORSE_TIME_3 50
#define RAMP_END 1500
#define P_FACTOR 0.5
#define I_FACTOR 0.2
#define GEAR_RATIO 12L
#define PAS_THRESHOLD 1.9
#define RAMP_START 64000
#define limit_with_throttle_override 35
#define CORRECTION_AT_ANGLE 127
#define ANGLE_4_0 1
#define ANGLE_6_60 38
#define ANGLE_2_120 81
#define ANGLE_3_180 111
#define ANGLE_1_240 180
#define ANGLE_5_300 208

#define NGLE_4_0 1
#define NGLE_6_60 38
#define NGLE_2_120 82
#define NGLE_3_180 128
#define NGLE_1_240 166
#define NGLE_5_300 210

#define TQS_CALIB 0.0
#define ACA 4764 
#define EEPROM_INIT_MAGIC_BYTE 173 // makes sure (chance of fail 1/255) eeprom is invalidated after flashing new config
#define ADC_BATTERY_VOLTAGE_K 69
#define ACA_EXPERIMENTAL 128
#define BATTERY_VOLTAGE_MAX_VALUE 110
#define WALK_ASSIST_SPEED_LIMIT 6
#define WALK_ASSIST_CURRENT_TARGET 5

#define USE_FIELD_WEAKENING
#define SWAP_PHASES 0
/*
 * pick one
 */

#define DISPLAY_TYPE_KT_LCD8
//#define DISPLAY_TYPE_KT_LCD3
//#define DIAGNOSTICS
//#define BLUOSEC
//#define TT
//
#endif /* CONFIG_H_ */
