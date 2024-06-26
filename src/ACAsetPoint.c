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
 * File:   ACAsetPoint.c
 * Author: Björn Schmidt
 *
 * Created on September 7, 2018, 6:47 PM
 */

#include <stdio.h>
#include <stdint.h>
#include "config.h"
#include "stm8s.h"
#include "stm8s_it.h"
#include "ACAsetPoint.h"
#include "ACAcontrollerState.h"
#include "ACAcommons.h"
// FIXME ugly cross references
// why? cause this blackbox is just for calculating,
// it's not supposed to read values on its own
// all values should be read by a slowloop_"readall/updatesensors, whatever" before calling it
#include "brake.h"
#include "adc.h" // FIXME ugly cross reference

uint32_t ui32_dutycycle; // local version of setpoint

static int8_t uint_PWM_Enable = 0; //flag for PWM state
uint16_t ui16_BatteryCurrent_accumulated = 2496L; //8x current offset, for filtering or Battery Current
uint16_t ui16_BatteryVoltage_accumulated;
static uint16_t ui16_assist_percent_smoothed;
static uint32_t ui32_time_ticks_between_pas_interrupt_accumulated = 0; // for filtering of PAS value 
uint32_t ui32_erps_accumulated; //for filtering of erps
//static uint32_t ui32_speedlimit_actual_accumulated;
static uint32_t ui32_sumthrottle_accumulated; //it is already smoothed b4 we get it, we want to smooth it even more though for dynamic assist levels

static float float_temp = 0; //for float calculations

static uint32_t uint32_temp = 0;
static uint16_t uint16_temp = 0;
static uint16_t controll_state_temp = 0;
static uint8_t ui8_temp = 0;

uint16_t cutoffSetpoint(uint32_t ui32_dutycycle) {
	if (ui32_dutycycle < 5) {
		ui32_dutycycle = 0;
	}
	if (ui32_dutycycle > 255) {
		ui32_dutycycle = 255;
	}
	return ui32_dutycycle;
}

BitStatus checkMaxErpsOverride(){
	if (ui32_erps_filtered > ui16_erps_max) {
		/* ui32_dutycycle = PI_control(ui32_erps_filtered, ui16_erps_max,uint_PWM_Enable); //limit the erps to maximum value to have minimum 30 points of sine table for proper commutation */
		ui32_dutycycle = PI_control_fixed(ui32_erps_filtered, ui16_erps_max,uint_PWM_Enable); //limit the erps to maximum value to have minimum 30 points of sine table for proper commutation
		controll_state_temp +=1024;
		return 1;
	}
	return 0;
}
int test = 1;
BitStatus checkUnderVoltageOverride(){
	//check for undervoltage --> ramp down power starting 6.25% above min
	ui8_temp = ui8_s_battery_voltage_min + (ui8_s_battery_voltage_min>>4);
	if (ui8_BatteryVoltage < ui8_temp) {
		uint32_current_target = (uint32_t) map16o8i(ui8_BatteryVoltage, ui8_s_battery_voltage_min, ui8_temp, ui16_current_cal_b, (uint16_t) uint32_current_target ); // has to be < 1024 since that's the ADC max
		/* uint32_current_target = (uint32_t) map32(ui8_BatteryVoltage, ui8_s_battery_voltage_min, ui8_temp, ui16_current_cal_b, (uint16_t) uint32_current_target ); // has to be < 1024 since that's the ADC max */
        /* ui32_dutycycle = PI_control(ui16_BatteryCurrent, uint32_current_target,uint_PWM_Enable); */
        ui32_dutycycle = PI_control_fixed(ui16_BatteryCurrent, uint32_current_target,uint_PWM_Enable);
		controll_state_temp +=2048;
		return 1;
    }
	return 0;
}

BitStatus checkOverVoltageOverride(){
	//check for overvoltage --> ramp down regen starting 3.125% below max
	ui8_temp = ui8_s_battery_voltage_max - (ui8_s_battery_voltage_max>>5);
	if (ui8_BatteryVoltage > ui8_temp) {
		/* uint32_current_target = (uint32_t) map32(ui8_BatteryVoltage, ui8_temp, ui8_s_battery_voltage_max, (uint16_t) uint32_current_target, ui16_current_cal_b ); */
		uint32_current_target = (uint32_t) map16o8i(ui8_BatteryVoltage, ui8_temp, ui8_s_battery_voltage_max, (uint16_t) uint32_current_target, ui16_current_cal_b );
		/* ui32_dutycycle = PI_control(ui16_BatteryCurrent, uint32_current_target,uint_PWM_Enable); */
		ui32_dutycycle = PI_control_fixed(ui16_BatteryCurrent, uint32_current_target,uint_PWM_Enable);
		controll_state_temp +=4096;
		return 1;
    }
	return 0;
}

void aca_setpoint_init(void) {
	ui32_time_ticks_between_pas_interrupt_accumulated = ((uint32_t)ui16_s_ramp_start)<<3;
}

uint16_t aca_setpoint(uint16_t ui16_time_ticks_between_pas_interrupt, uint16_t setpoint_old) {
	// select virtual erps speed based on speedsensor type
    //

	if (((ui8_aca_flags_high & EXTERNAL_SPEED_SENSOR) == EXTERNAL_SPEED_SENSOR)) {
		ui16_virtual_erps_speed = (uint16_t) ((((uint32_t)ui8_gear_ratio) * ui32_speed_sensor_rpks) /1000); 
	}else{
		ui16_virtual_erps_speed = (uint16_t) ui32_erps_filtered;
	}

	// first select current speed limit
	if (ui8_offroad_state == 255) {
		ui8_speedlimit_actual_kph = 80;
	} else if (ui8_walk_assist || ((ui8_aca_experimental_flags_high & THROTTLE_ALLOWED_FOR_WALK) && ui16_sum_throttle > 2)) {
		ui8_speedlimit_actual_kph = WALK_ASSIST_SPEED_LIMIT;
        ui8_walk_assist = 1;
	} else if (ui8_offroad_state > 15 && ui16_sum_throttle <= 2) { // allow a slight increase based on ui8_offroad_state
		ui8_speedlimit_actual_kph = ui8_speedlimit_kph + (ui8_offroad_state - 16);
	} else if (ui8_offroad_state > 15 && ui16_sum_throttle > 2) {
		ui8_speedlimit_actual_kph = ui8_speedlimit_with_throttle_override_kph + (ui8_offroad_state - 16);
    } else if ((ui8_aca_experimental_flags_high & THROTTLE_UNRESTRICTED) && ui16_sum_throttle > 2) {
        ui8_speedlimit_actual_kph = ui8_speedlimit_with_throttle_override_kph;
	} else if (ui16_time_ticks_for_pas_calculation > timeout || !PAS_is_active) {
		ui8_speedlimit_actual_kph = ui8_speedlimit_without_pas_kph;
	} else {
		ui8_speedlimit_actual_kph = ui8_speedlimit_kph;
	}
	// >=8 means levels are switched of, use wanted percentage directly instead
	ui16_assist_percent_smoothed -= ui16_assist_percent_smoothed >> 4;
	if ((ui8_assistlevel_global & 15) < 8) {
		ui16_assist_percent_smoothed += ui8_a_s_assistlevels[ui8_assistlevel_global & 15];
	} else {
		ui16_assist_percent_smoothed += ui8_assist_percent_wanted;
	}
	ui8_assist_percent_actual = ui16_assist_percent_smoothed >> 4;

	// average throttle over a longer time period (for dynamic assist level) 
	ui32_sumthrottle_accumulated -= ui32_sumthrottle_accumulated >> 10;
	ui32_sumthrottle_accumulated += ui16_sum_throttle;
	ui8_assist_dynamic_percent_addon = ui32_sumthrottle_accumulated >> 10;
	if ((ui8_assist_dynamic_percent_addon + ui8_assist_percent_actual) > 100) {
		ui8_assist_dynamic_percent_addon = 100 - ui8_assist_percent_actual;
	}

	ui16_BatteryCurrent_accumulated -= ui16_BatteryCurrent_accumulated >> 3;
	ui16_BatteryCurrent_accumulated += ui16_adc_read_motor_total_current();
	ui16_BatteryCurrent = ui16_BatteryCurrent_accumulated >> 3;

	ui16_BatteryVoltage_accumulated -= ui16_BatteryVoltage_accumulated >> 3;
	ui16_BatteryVoltage_accumulated += ui8_adc_read_battery_voltage();
	ui8_BatteryVoltage = ui16_BatteryVoltage_accumulated >> 3;

	ui32_erps_accumulated -= ui32_erps_accumulated >> 3;
	ui32_erps_accumulated += ui16_motor_speed_erps;
	ui32_erps_filtered = ui32_erps_accumulated >> 3;

	ui32_time_ticks_between_pas_interrupt_accumulated -= ui32_time_ticks_between_pas_interrupt_accumulated >> 3;
	// do not allow values > ramp_start into smoothing cause it makes startup sluggish
	// also do not allow values < ramp_start when pedalling backwards
	if ((!PAS_is_active||(ui16_time_ticks_between_pas_interrupt > ui16_s_ramp_start)) && ((ui8_aca_flags_high & TQ_SENSOR_MODE) != TQ_SENSOR_MODE)) {
		ui32_time_ticks_between_pas_interrupt_accumulated += ui16_s_ramp_start;
	} else {
		ui32_time_ticks_between_pas_interrupt_accumulated += ui16_time_ticks_between_pas_interrupt;
	}
	ui16_time_ticks_between_pas_interrupt_smoothed = ui32_time_ticks_between_pas_interrupt_accumulated >> 3;

	ui8_moving_indication = 0;
	// check for brake --> set regen current
	if (brake_is_set()) { // float code takes at worst 3.12ms, uint32 code 660ms (both worst code paths)
		ui8_cruiseThrottleSetting = 0;
		ui8_moving_indication |= (32);
		controll_state_temp = 255;
        
		//Current target based on regen assist level
		if ((ui8_aca_flags_low & DIGITAL_REGEN) == DIGITAL_REGEN) {

			ui8_temp = ui8_a_s_assistlevels[ui8_assistlevel_global >> 4];
			controll_state_temp -= 1;

			//Current target based on linear input on pad X4
		} else {
            if (ui8_aca_experimental_flags_high & THROTTLE_REGEN) {
			    ui8_temp = map8u(ui8_momentary_throttle, ui8_throttle_min_range, ui8_throttle_max_range, 0, 128); //use throttle to vary regen when braking
            } else {
#ifndef X4_TEMPERATURE
                ui8_temp = map8u(ui16_x4_value >> 2, ui8_throttle_min_range, ui8_throttle_max_range, 0, 128); //map regen throttle to limits
#else
                ui8_temp = 1;
#endif
            }
			controll_state_temp -= 2;
		}
        uint32_t ui32_temp = ((uint32_t)ui8_temp * (uint32_t) ui16_regen_current_max_value);
        ui32_temp >>= 7;

		//Current target gets ramped down with speed
		if (((ui8_aca_flags_low & SPEED_INFLUENCES_REGEN) == SPEED_INFLUENCES_REGEN) && (ui16_virtual_erps_speed < ((ui16_speed_kph_to_erps_ratio * ((uint16_t) ui8_speedlimit_kph)) / 100))) {
            ui32_temp *= ui16_virtual_erps_speed;
            ui32_temp <<= 8; // 24.8
            ui32_temp /= ((uint32_t) ui16_speed_kph_to_erps_ratio * (uint32_t) ui8_speedlimit_kph);
            ui32_temp *= 100;
            ui32_temp >>= 8; // rouding can offset by one but we ignore it 
            controll_state_temp -= 4;
		}

		uint32_current_target = ui16_current_cal_b - ui32_temp;


		if (!checkOverVoltageOverride()){
			/* ui32_dutycycle = PI_control(ui16_BatteryCurrent, uint32_current_target,uint_PWM_Enable); */
			ui32_dutycycle = PI_control_fixed(ui16_BatteryCurrent, uint32_current_target,uint_PWM_Enable);
		}
		
		if (((ui8_aca_flags_high & BYPASS_LOW_SPEED_REGEN_PI_CONTROL) == BYPASS_LOW_SPEED_REGEN_PI_CONTROL) && (ui32_dutycycle == 0)) {
			//try to get best regen at Low Speeds for BionX IGH
			ui32_dutycycle = ui16_virtual_erps_speed * 2;
			controll_state_temp -= 8;
		}
	} else {
		uint32_current_target = ui16_current_cal_b; // reset target to zero
		controll_state_temp = 0;
		//if none of the overruling boundaries are concerned, calculate new setpoint

		// if torque sim is requested. We could check if we could solve this function with just one line with map function...
		if (((ui8_aca_flags_high & TQ_SENSOR_MODE) != TQ_SENSOR_MODE)) {

			// add dynamic assist level based on past throttle input
			ui8_temp = ui8_assist_percent_actual;

			if ((ui8_aca_flags_high & DYNAMIC_ASSIST_LEVEL) == DYNAMIC_ASSIST_LEVEL) {
				ui8_temp += ui8_assist_dynamic_percent_addon;
			}

			if (ui16_time_ticks_between_pas_interrupt_smoothed > ui16_s_ramp_end) { //ramp end usually 1500
				//if you are pedaling slower than defined ramp end
				//but faster than ramp start
				// if pedaling slower then ramp start, the current target stays at current cal b
				//current is proportional to cadence
				if (ui16_time_ticks_between_pas_interrupt_smoothed < ui16_s_ramp_start) {
					uint32_current_target = (ui8_temp * (ui16_battery_current_max_value) / 100);

                    // precision loss is small, taks half the time (about 400us vs 850us) vs floats
                    uint32_temp = ((uint32_t)(ui16_time_ticks_between_pas_interrupt_smoothed - ui16_s_ramp_end)) << 16; // avoid aliasing
                    uint32_temp /= ((ui16_s_ramp_start - ui16_s_ramp_end)); // note: less than "0"
                    uint32_temp = 65536 - uint32_temp; // (1.0 - value)
                    uint32_current_target = ((uint32_current_target * uint32_temp) >> 16) + ui16_current_cal_b; // note, since it's a adc value it's always going to be <= 16bit
                                                                                                                //
					controll_state_temp += 1;
					ui8_moving_indication |= (16);
					ui8_cruiseThrottleSetting = 0; //no cruise when pedalling, just like stock
				}
				//in you are pedaling faster than in ramp end defined, desired battery current level is set,
			} else {
				uint32_current_target = (ui8_temp * (ui16_battery_current_max_value) / 100 + ui16_current_cal_b);
				controll_state_temp += 2;
				ui8_moving_indication |= (16);
				ui8_cruiseThrottleSetting = 0; //no cruise when pedalling, just like stock
			}
		} else { // torque sensor mode

            uint32_temp = ui16_sum_torque;
            uint32_temp *= ui8_assist_percent_actual;
            uint32_temp *= ui16_battery_current_max_value;
            uint32_temp *= uint32_torquesensorCalibration;

            uint32_temp /= ui16_time_ticks_between_pas_interrupt_smoothed; // hier lässt sich die geteilt-Operation nicht vermeiden :-(

			//increase power linear with speed for convenient commuting :-)
			if ((ui8_aca_flags_low & SPEED_INFLUENCES_TORQUESENSOR) == SPEED_INFLUENCES_TORQUESENSOR) {
				uint32_temp *= 1 + (ui16_virtual_erps_speed) / ((ui16_speed_kph_to_erps_ratio * (ui8_speedlimit_kph)) / 100); // influence of current speed based on base speed limit
			}

            if (PAS_is_active) {
                uint32_current_target = (uint32_temp >>8) +  (uint32_t) ui16_current_cal_b; //right shift 15 fasst die Operationen /100 (annähernd >>7) aus der assist_percent und /255 ( >>8) aus dem battery_current max zusammen, ist nicht ganz korrekt, ggf. nur >>14 nehmen -->/(256*128) vs. /(256*64)
            } else {
                uint32_current_target =(uint32_t) ui16_current_cal_b;
            }

			controll_state_temp += 4;

		}

        // float takes 1.42ms, uint16 takes 0.72ms. 
        uint16_t ui16_temp = 0;
		// throttle / torquesensor override following
		if (((ui8_aca_flags_high & TQ_SENSOR_MODE) != TQ_SENSOR_MODE)) {
			if (ui8_speedlimit_kph > 1) {
				// do not apply throttle at very low speed limits (technical restriction, speelimit can and should never be lover than 1)
				if (ui8_cruiseThrottleSetting >= 25) {
					ui8_moving_indication |= (8);
                    ui16_temp = ui8_cruiseThrottleSetting << 8;
				}
				else {
					ui8_cruiseThrottleSetting = 0;
                    ui16_temp = ui8_momentary_throttle << 8;
				}
			}
		} else {
		
            ui16_temp = ui8_momentary_throttle << 8;

			//float_temp *= (1 - (float) ui16_virtual_erps_speed / 2 / (float) (ui16_speed_kph_to_erps_ratio/100 * ((float) ui8_speedlimit_kph))); //ramp down linear with speed. Risk: Value is getting negative if speed>2*speedlimit
			// above line wasnt working anyway before, so I commented it out, but it should be fixed with the division by 100
		}

		// map curret target to assist level, not to maximum value
		if ((ui8_aca_flags_low & ASSIST_LVL_AFFECTS_THROTTLE) == ASSIST_LVL_AFFECTS_THROTTLE) {
            ui16_temp /= 100;
            ui16_temp *= ui8_assist_percent_actual;
			controll_state_temp += 8;
		}

#if BATTERY_CURRENT_MAX_VALUE > 255
    #error BATTERY_CURRENT_MAX_VALUE cannot be higher than 255 in this version of the code due to optimizations
#endif
        if (ui16_temp & 0x00FF > 127) ui16_temp += (1 << 8); // if >= 0.5 round up
        ui16_temp >>= 8;
        ui16_temp *= ui16_battery_current_max_value; // at 25A this would be ~250, we error out if BATTERY_CURRENT_MAX_VALUE > 255
        ui16_temp >>= 8; // decrease precision again, ignore rounding errors
        ui16_temp += ui16_current_cal_b;


		if ((uint32_t) ui16_temp > uint32_current_target) {
			if (((ui8_aca_flags_high & TQ_SENSOR_MODE) == TQ_SENSOR_MODE)) {
				if (uint32_current_target > ui16_current_cal_b){
					//override cadence based torque with torquesensor-throttle only if there is cadence based contribution
					uint32_current_target = (uint32_t) ui16_temp;
					ui8_moving_indication &= ~(16);
					ui8_moving_indication |= 2;
				}
			}else{
				//override torque simulation with throttle
				uint32_current_target = (uint32_t) ui16_temp; 
				ui8_moving_indication &= ~(16);
				ui8_moving_indication |= 2;
			}
			controll_state_temp += 16;
		}

		// check for overspeed
		uint32_temp = uint32_current_target;
		uint32_current_target = CheckSpeed((uint16_t) uint32_current_target, (uint16_t) ui16_virtual_erps_speed, (ui16_speed_kph_to_erps_ratio * ((uint16_t) ui8_speedlimit_actual_kph)) / 100, (ui16_speed_kph_to_erps_ratio * ((uint16_t) (ui8_speedlimit_actual_kph + 2))) / 100); //limit speed
                                                                                                                                                                                                                                                                                      //
		if (uint32_temp != uint32_current_target) {
			controll_state_temp += 32;
		}

		if (uint32_current_target > ui16_battery_current_max_value + ui16_current_cal_b) {
			uint32_current_target = ui16_battery_current_max_value + ui16_current_cal_b;
			controll_state_temp += 64;
		}
		//phase current limiting
		if (setpoint_old > 0 && (uint32_current_target - ui16_current_cal_b)*255 / setpoint_old > PHASE_CURRENT_MAX_VALUE) { // limit phase current according to Phase Current = battery current/duty cycle
			uint32_current_target = (PHASE_CURRENT_MAX_VALUE) * setpoint_old / 255 + ui16_current_cal_b;
			controll_state_temp += 128;
		}


		if ((ui8_aca_experimental_flags_low & DC_STATIC_ZERO) == DC_STATIC_ZERO) {
			ui32_dutycycle = 0;
			controll_state_temp += 256;
            // next took 1.94ms, after opti of mapping 0.26ms
		} else if (!checkUnderVoltageOverride() && !checkMaxErpsOverride()){

			if (ui8_walk_assist) uint32_current_target = (WALK_ASSIST_CURRENT_TARGET) + ui16_current_cal_b;
			else {
				// control power instead of current
				if ((ui8_aca_flags_high & POWER_BASED_CONTROL) == POWER_BASED_CONTROL) {
					// nominal voltage based on limits
					ui8_temp = ((ui8_s_battery_voltage_max - ui8_s_battery_voltage_min) >> 1) + ui8_s_battery_voltage_min;
					//uint32_current_target*=ui8_temp/ui8_BatteryVoltage;
					if (ui8_moving_indication & 16 == 16) {
						uint32_current_target -= ui16_current_cal_b;
						uint32_current_target *= ui8_temp; // or nominal voltage at which you want to calculate the power target
						uint32_current_target /= ui8_BatteryVoltage;
						uint32_current_target += ui16_current_cal_b;
					}
				}
			}

			//send current target to PI-controller
			/* ui32_dutycycle = PI_control(ui16_BatteryCurrent, uint32_current_target,uint_PWM_Enable); */
			ui32_dutycycle = PI_control_fixed(ui16_BatteryCurrent, uint32_current_target,uint_PWM_Enable);

		}
		
		if ((ui8_aca_experimental_flags_high & PWM_AUTO_OFF) == PWM_AUTO_OFF) {
			controll_state_temp += 512;
			//disable PWM if enabled and no power is wanted
			if (uint_PWM_Enable && ui32_erps_filtered == 0 && uint32_current_target == ui16_current_cal_b) {
				TIM1_CtrlPWMOutputs(DISABLE);
				uint_PWM_Enable = 0;
			}
			//enable PWM if disabled and voltage is 6.25% higher than min, some hysteresis and power is wanted
			if (!uint_PWM_Enable && ui8_BatteryVoltage > (ui8_s_battery_voltage_min +  (ui8_s_battery_voltage_min >>4)) && (uint32_current_target != ui16_current_cal_b)){
				TIM1_CtrlPWMOutputs(ENABLE);
				uint_PWM_Enable = 1;
			}
		}else{

			//enable PWM if disabled and voltage is 6.25% higher than min, some hysteresis
			if (!uint_PWM_Enable && ui8_BatteryVoltage > (ui8_s_battery_voltage_min + (ui8_s_battery_voltage_min >>4))) { 
				TIM1_CtrlPWMOutputs(ENABLE);
				uint_PWM_Enable = 1;
			}
		}
	}
	ui16_control_state = controll_state_temp;
	return cutoffSetpoint(ui32_dutycycle);

}
