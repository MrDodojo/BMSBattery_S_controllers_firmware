/*
 * config.h
 *
 *  Automatically created by OSEC Parameter Configurator
 *  Author: stancecoke
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define NUMBER_OF_PAS_MAGS 10
#define limit 26
#define timeout 2048
#define wheel_circumference 2268L
#define limit_without_pas 4
#define ADC_THROTTLE_MIN_VALUE 44
#define ADC_THROTTLE_MAX_VALUE 178
#define BATTERY_VOLTAGE_MIN_VALUE 143
#define BATTERY_CURRENT_MAX_VALUE 20L
#define PHASE_CURRENT_MAX_VALUE 500L
#define REGEN_CURRENT_MAX_VALUE 41L
#define MOTOR_ROTOR_DELTA_PHASE_ANGLE_RIGHT 237
#define current_cal_a 34
#define LEVEL_1 38
#define LEVEL_2 54
#define LEVEL_3 69
#define LEVEL_4 85
#define LEVEL_5 100
#define MORSE_TIME_1 85
#define MORSE_TIME_2 35
#define MORSE_TIME_3 10
#define RAMP_END 1500
#define P_FACTOR 0.5
#define I_FACTOR 0.1
#define GEAR_RATIO 23L
#define PAS_THRESHOLD 1.9
#define RAMP_START 64000
#define limit_with_throttle_override 80
#define CORRECTION_AT_ANGLE 127
#define DISPLAY_TYPE_KT_LCD3
#define ANGLE_4_0 1
#define ANGLE_6_60 43
#define ANGLE_2_120 86
#define ANGLE_3_180 128
#define ANGLE_1_240 171
#define ANGLE_5_300 213
#define TQS_CALIB 0.0
#define ACA 5762
#define EEPROM_INIT_MAGIC_BYTE 236 // makes sure (chance of fail 1/255) eeprom is invalidated after flashing new config
#define ADC_BATTERY_VOLTAGE_K 69
#define ACA_EXPERIMENTAL 1152
#define BATTERY_VOLTAGE_MAX_VALUE 219

#endif /* CONFIG_H_ */
