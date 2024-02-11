/*
 * Copyright (c) 2018 Björn Schmidt
 *
 * This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
 */

/* 
 * File:   ACAcontrollerState.h
 * Author: Björn Schmidt
 *
 * Created on August 30, 2018, 8:03 PM
 */


#ifndef ACACONTROLLERSTATE_H
#define ACACONTROLLERSTATE_H

#include "config.h"

extern uint32_t uint32_icc_signals; // inter component communication, very simplistig way of signalling stuff via a shared var

extern uint16_t ui16_erps_max;
extern uint16_t ui16_pwm_cycles_second;

extern uint8_t ui8_throttle_min_range;
extern uint8_t ui8_throttle_max_range;
extern uint16_t ui16_control_state;
extern uint8_t ui8_a_s_assistlevels[6];
extern uint8_t ui8_assist_percent_actual;
extern uint8_t ui8_assist_percent_wanted;
extern uint8_t ui8_assistlevel_global; //assist level for regen (high 4 bits) and torque (low 4 bits)
extern uint8_t ui8_walk_assist;
extern uint8_t ui8_assist_dynamic_percent_addon;
extern uint8_t PAS_act;
extern uint8_t PAS_is_active;
extern uint16_t ui16_sum_torque;
extern uint16_t ui16_sum_throttle;
extern uint16_t ui8_momentary_throttle;
extern uint8_t ui8_offroad_state;
extern uint32_t uint32_current_target;
extern uint16_t ui16_setpoint;
extern uint16_t ui16_throttle_accumulated;
extern uint16_t ui16_current_cal_b;
extern uint8_t ui8_current_cal_a;
extern uint16_t ui16_x4_cal_b;
extern uint16_t ui16_x4_value;
extern uint16_t ui16_throttle_cal_b;
extern uint16_t ui16_battery_current_max_value;
extern uint16_t ui16_regen_current_max_value;
extern uint8_t ui8_possible_motor_state;
extern uint8_t ui8_dynamic_motor_state;
extern uint8_t ui8_BatteryVoltage;
extern uint8_t ui8_battery_voltage_nominal;
extern uint16_t ui16_motor_speed_erps;
extern uint16_t ui16_virtual_erps_speed;
extern uint32_t ui32_erps_filtered; //filtered value of erps
extern uint16_t ui16_BatteryCurrent;
extern uint8_t ui8_position_correction_value;
extern uint8_t ui8_correction_at_angle;
extern uint16_t ui16_ADC_iq_current;
extern uint16_t ui16_ADC_iq_current_filtered;
extern uint16_t ui16_ADC_iq_current_accumulated;
extern uint8_t ui8_speedlimit_kph;
extern uint8_t ui8_speedlimit_without_pas_kph;
extern uint8_t ui8_speedlimit_actual_kph;
extern uint8_t ui8_speedlimit_with_throttle_override_kph;
extern uint8_t ui8_uptime;

extern uint8_t ui8_variableDebugA;
extern uint8_t ui8_variableDebugB;
extern uint8_t ui8_variableDebugC;

extern int8_t i8_motor_temperature;

extern uint8_t uint8_t_60deg_pwm_cycles[6];
extern uint8_t uint8_t_hall_case[7];
extern uint8_t uint8_t_hall_order[6];
extern int8_t int8_t_hall_counter;
extern uint8_t current_hall;
extern uint8_t ui8_hall_debug_counter;
extern uint8_t ui8_hall_order_counter;

extern uint8_t ui8_gear_ratio;
extern uint16_t ui16_speed_kph_to_erps_ratio;

extern uint32_t ui32_speed_sensor_rpks; //speed sensor rounds per 1000 sec
extern uint32_t ui32_speed_sensor_rpks_accumulated;

extern uint16_t ui16_time_ticks_for_uart_timeout;
extern uint16_t ui16_time_ticks_for_speed_calculation;
extern uint16_t ui16_time_ticks_between_speed_interrupt; //Counter for bike speed
extern uint8_t ui8_SPEED_Flag; //Flag for PAS Interrupt detected
extern uint16_t ui16_time_ticks_between_speed_interrupt; //Speed duration of one wheel revolution (tics * 64us)
extern uint8_t ui8_offroad_counter;
extern uint16_t ui16_idle_counter;
extern uint16_t ui16_no_pass_counter;
extern uint16_t ui16_passcode;
extern uint8_t ui8_lockstatus;

extern uint8_t ui8_aca_flags_low;
extern uint8_t ui8_aca_flags_high;
extern uint8_t ui8_aca_experimental_flags_low;
extern uint8_t ui8_aca_experimental_flags_high;

extern uint16_t ui16_torque[NUMBER_OF_PAS_MAGS];
extern uint8_t ui8_torque_index;

extern uint32_t uint32_torquesensorCalibration;

extern uint16_t ui16_time_ticks_between_pas_interrupt_smoothed; // for filtering of PAS value
extern float flt_current_PAS_fraction;
extern uint16_t ui16_time_ticks_between_pas_interrupt;
extern uint16_t ui16_PAS_High;
extern uint16_t ui16_time_ticks_for_pas_calculation; //Counter for cadence
extern uint16_t ui16_PAS_High_Counter;
extern uint8_t ui8_PAS_Flag; //flag for PAS interrupt
extern uint8_t ui8_PAS_update_call_when_inactive_counter;


extern float flt_torquesensorCalibration;
extern float flt_s_pas_threshold;
extern float flt_s_pid_gain_p;
extern float flt_s_pid_gain_i;
extern float flt_s_motor_constant;
extern uint16_t ui16_s_ramp_end;
extern uint16_t ui16_s_ramp_start;
extern uint8_t ui8_s_motor_angle;
extern uint8_t ui8_s_hall_angle4_0;
extern uint8_t ui8_s_hall_angle6_60;
extern uint8_t ui8_s_hall_angle2_120;
extern uint8_t ui8_s_hall_angle3_180;
extern uint8_t ui8_s_hall_angle1_240;
extern uint8_t ui8_s_hall_angle5_300;

extern uint8_t ui8_s_battery_voltage_calibration;
extern uint8_t ui8_s_battery_voltage_min;
extern uint8_t ui8_s_battery_voltage_max;

extern uint8_t light_stat;
extern uint8_t walk_stat;

extern uint8_t ui8_moving_indication;
extern uint8_t ui8_cruiseThrottleSetting;
extern uint8_t ui8_cruiseMinThrottle;

extern uint8_t pwm_swap_phases;

void controllerstate_init(void);

typedef enum {
	// values from 0-31 are allowed as signals are stored in a single uint32_t
	SIGNAL_SPEEDLIMIT_CHANGED = ((uint8_t) 0x00),

} ICC_SIGNALS;

typedef enum {
	ASSIST_LVL_AFFECTS_THROTTLE = ((uint8_t) 0x01),
	OFFROAD_ENABLED = ((uint8_t) 0x02),
	BRAKE_DISABLES_OFFROAD = ((uint8_t) 0x04),
	IDLE_DISABLES_OFFROAD = ((uint8_t) 0x08),

	DIGITAL_REGEN = ((uint8_t) 0x10),
	SPEED_INFLUENCES_REGEN = ((uint8_t) 0x20),
	SPEED_INFLUENCES_TORQUESENSOR = ((uint8_t) 0x40),

	PAS_INVERTED = ((uint8_t) 0x80),
			
} ACA_FLAGS_LOW;

typedef enum {
	DUMMY_ALWAYS_ON = ((uint8_t) 0x01),
	BYPASS_LOW_SPEED_REGEN_PI_CONTROL = ((uint8_t) 0x02),
	DYNAMIC_ASSIST_LEVEL = ((uint8_t) 0x04),
	POWER_BASED_CONTROL= ((uint8_t) 0x08),
	TQ_SENSOR_MODE = ((uint8_t) 0x10),
	ANGLE_CORRECTION_ENABLED = ((uint8_t) 0x20),
	EXTERNAL_SPEED_SENSOR = ((uint8_t) 0x40),	
} ACA_FLAGS_HIGH;

typedef enum {

	DC_STATIC_ZERO = ((uint8_t) 0x01),
	AVOID_MOTOR_CYCLES_JITTER = ((uint8_t) 0x02),
	DISABLE_INTERPOLATION = ((uint8_t) 0x04),
	DISABLE_60_DEG_INTERPOLATION = ((uint8_t) 0x08),
	SWITCH_360_DEG_INTERPOLATION = ((uint8_t) 0x10),
	USE_ALTERNATE_WAVETABLE = ((uint8_t) 0x20),
	USE_ALTERNATE_WAVETABLE_B = ((uint8_t) 0x40),
    USE_ALTERNATE_WAVETABLE_C = ((uint8_t) 0x80),
	
} ACA_EXPERIMENTAL_FLAGS_LOW;

typedef enum {
    HIGH_SPEED_MOTOR = ((uint8_t) 0x01),
	PWM_AUTO_OFF = ((uint8_t) 0x02),
    DUMMY_EXP_ALWAYS_ON = ((uint8_t) 0x04),		
    THROTTLE_REGEN = ((uint8_t) 0x08),
    THROTTLE_ALLOWED_FOR_WALK = ((uint8_t) 0x10),
    THROTTLE_UNRESTRICTED = ((uint8_t) 0x20),
} ACA_EXPERIMENTAL_FLAGS_HIGH;

#define ACA_LOW  (BRAKE_DISABLES_OFFROAD | DIGITAL_REGEN | SPEED_INFLUENCES_REGEN)
#define ACA_HIGH (DYNAMIC_ASSIST_LEVEL | ANGLE_CORRECTION_ENABLED)
#define ACA_EXPERIMENTAL_LOW (USE_ALTERNATE_WAVETABLE_C | USE_ALTERNATE_WAVETABLE)
#define ACA_EXPERIMENTAL_HIGH (PWM_AUTO_OFF)
#endif /* BOCONTROLLERSTATE_H */

