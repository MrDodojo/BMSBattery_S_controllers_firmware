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
 * File:   ACAcommons.c
 * Author: Björn Schmidt
 *
 * Created on September 7, 2018, 7:31 PM
 */

#include <stdio.h>
#include <stdint.h>
#include "stm8s.h"
#include "stm8s_itc.h"
#include "stm8s_gpio.h"
#include "config.h"
#include "gpio.h"
#include "adc.h"
#include "ACAcommons.h"
#include "ACAcontrollerState.h"

static uint16_t ui16_temp;
static uint8_t ui8_temp;

uint8_t float2int(float in, float maxRange) {
	uint16_t result;
	if (in < 0.0)
		return 0;
	result = (uint16_t) (in * (float) ((float) 256 / (float) maxRange));
	if (result > 255)
		result = 255;
	return (result);
}

float int2float(uint8_t in, float maxRange) {
	return ((float) in / (float) ((float) 256 / (float) maxRange));
}

/*
 * since we need only achieve 16bit accuracy, the map functions can be sped up ba an order of magnitude (200 -> 20us)
 * if an 8bit value is needed
 */
uint8_t map8u(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max) {
	// if input is smaller/bigger than expected return the min/max out ranges value
	if (x < in_min)
		return out_min;
	else if (x > in_max)
		return out_max;

		// map the input to the output range.
		// round up if mapping bigger ranges to smaller ranges
	else if ((in_max - in_min) > (out_max - out_min))
		return ((uint16_t)(x - in_min)) * ((uint16_t) (out_max - out_min + 1)) / ((uint16_t)(in_max - in_min + 1)) + (uint16_t)out_min;
		// round down if mapping smaller ranges to bigger ranges
	else
		return ((uint16_t)(x - in_min)) * ((uint16_t) (out_max - out_min))     / ((uint16_t)(in_max - in_min))     + (uint16_t)out_min;
}

uint8_t map8ur(uint16_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max) {
	// if input is smaller/bigger than expected return the min/max out ranges value
	if (x < in_min)
		return out_min;
	else if (x > in_max)
		return out_max;

		// map the input to the output range.
		// round up if mapping bigger ranges to smaller ranges
	else if ((in_max - in_min) > (out_max - out_min))
		return ((uint16_t)(x - in_min)) * ((uint16_t) (out_max - out_min + 1)) / ((uint16_t)(in_max - in_min + 1)) + (uint16_t) out_min;
		// round down if mapping smaller ranges to bigger ranges
	else
		return ((uint16_t)(x - in_min)) * ((uint16_t) (out_max - out_min))     / ((uint16_t)(in_max - in_min))     + (uint16_t) out_min;
}

uint16_t map16o8i(uint8_t x, uint8_t in_min, uint8_t in_max, uint16_t out_min, uint16_t out_max) {
	// if input is smaller/bigger than expected return the min/max out ranges value
	if (x < in_min) 
		return out_min;
	else if (x > in_max)
		return out_max;

		// map the input to the output range.
		// round up if mapping bigger ranges to smaller ranges
	else if (((uint16_t)(in_max - in_min)) > (out_max - out_min))
		return ((uint16_t)(x - in_min)) * ((uint16_t) (out_max - out_min + 1)) / ((uint16_t)(in_max - in_min + 1)) + (uint16_t)out_min;
		// round down if mapping smaller ranges to bigger ranges
	else
		return ((uint16_t)(x - in_min)) * ((uint16_t) (out_max - out_min))     / ((uint16_t)(in_max - in_min))     + (uint16_t)out_min;
}

int32_t map32(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
	// if input is smaller/bigger than expected return the min/max out ranges value
	if (x < in_min)
		return out_min;
	else if (x > in_max)
		return out_max;

		// map the input to the output range.
		// round up if mapping bigger ranges to smaller ranges
	else if ((in_max - in_min) > (out_max - out_min))
		return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
		// round down if mapping smaller ranges to bigger ranges
	else
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static int32_t fi, fdc = 0;
static float float_i, float_dc = 0;
/* static int32_t fi; */

void PI_control_jump(uint8_t dc) {
       float_i = dc;
       fi = dc * 256;
}

int32_t PI_control_fixed(uint16_t pv, uint16_t setpoint, uint8_t uint_PWM_Enable) {

    int32_t fp;
    static int32_t fi, fdc;

    fp = (((int32_t) ((int32_t)setpoint - (int32_t)pv))) * P_FACTOR_INT;
    fi += ((((int32_t) ((int32_t) setpoint - (int32_t)pv))) * I_FACTOR_INT);

	if (fi > (65280)) fi = (65280);
	if (fi < 0) fi = 0;

	if (!uint_PWM_Enable && ((ui8_aca_experimental_flags_high & PWM_AUTO_OFF) == PWM_AUTO_OFF)) {
        fi = (ui32_erps_filtered * 383) - fp; // mc = 1.5 always
	}



    if (fp + fi > fdc + (1280)) {
        fdc += 1280;
    } else if (fp + fi < fdc - (1280)) {
        fdc -= 1280;
    } else {
        fdc = fp + fi;
    }


    if (fdc > 65280) fdc = 65280;
	if (fdc < 0) fdc = 0;


	return ((uint8_t) (fdc >> 8));
}

int32_t PI_control(uint16_t pv, uint16_t setpoint, uint8_t uint_PWM_Enable) {
	float float_p;
	static float float_i;
	static float float_dc;

	float_p = ((float) setpoint - (float) pv) * flt_s_pid_gain_p;
	float_i += ((float) setpoint - (float) pv) * flt_s_pid_gain_i;

	if (float_i > 255) float_i = 255;
	if (float_i < 0) float_i = 0;

	if (!uint_PWM_Enable && ((ui8_aca_experimental_flags_high & PWM_AUTO_OFF) == PWM_AUTO_OFF)) {
		float_i = ui32_erps_filtered * flt_s_motor_constant - float_p;
	}

	if (float_p + float_i > float_dc + 5) {
		float_dc += 5;
	} else if (float_p + float_i < float_dc - 5) {
		float_dc -= 5;
	} else {
		float_dc = float_p + float_i;
	}

	if (float_dc > 255)float_dc = 255;
	if (float_dc < 0)float_dc = 0;

	return ((uint8_t) (float_dc));
}

void updateSpeeds(void) {
	// Update speed after speed interrupt occurrence
	if (ui8_SPEED_Flag) {
		if (ui16_time_ticks_for_speed_calculation > 1500) {
			//ignore spikes speed information, Do nothing if derived speed would be greater ca. 82km/h with 28" wheel
			ui16_time_ticks_between_speed_interrupt = ui16_time_ticks_for_speed_calculation; //save recent speed
			ui16_time_ticks_for_speed_calculation = 0; //reset speed counter

			ui32_speed_sensor_rpks_accumulated -= ui32_speed_sensor_rpks_accumulated >> 2;
			ui32_speed_sensor_rpks_accumulated += (((uint32_t)ui16_pwm_cycles_second)*1000) / ((uint32_t) ui16_time_ticks_between_speed_interrupt); // speed in rounds per 1000 seconds
			ui32_speed_sensor_rpks = ui32_speed_sensor_rpks_accumulated >> 2; //tic frequency 15625 Hz
		}

		ui8_SPEED_Flag = 0; //reset interrupt flag

	}
	//if wheel isn't turning, reset speed
	// FIXME, the following is gathered from two places that were executed just in that order
	// distinction 40000/65529 doesn't really make much sense
	if (ui16_time_ticks_for_speed_calculation > 40000) {
		ui32_speed_sensor_rpks = 0;
	}
	if (ui16_time_ticks_for_speed_calculation > 65529 && ui16_time_ticks_between_speed_interrupt != 65530) {
		ui16_time_ticks_between_speed_interrupt = 65530; //Set Display to 0 km/h
		PAS_act = 0; //Set PAS indicator to 0 to avoid motor startig, if pushing backwards from standstill
	}
}

uint32_t CheckSpeed(uint16_t current_target, uint16_t speed, uint16_t softLimit, uint16_t hardLimit) {
	//ramp down motor power if you are riding too fast and speed liming is active
	if (speed > softLimit && ui8_offroad_state != 255) { // FIXME the && part is only in here for old setpoint, aca already defines a higher limit

		if (speed > hardLimit) { //if you are riding much too fast, stop motor immediately
			current_target = ui16_current_cal_b;

		} else {
			uint32_t ui32_ct_normalized = ((current_target - ui16_current_cal_b));

			current_target = (uint16_t)
					((ui32_ct_normalized * (hardLimit - speed)) /
					(hardLimit - softLimit)
					+ ui16_current_cal_b);


		}
	}
	return ((uint32_t) current_target);
}

// internal signals so that components con communicate simple things without code cohesion

void setSignal(uint8_t signal) {
	uint32_icc_signals |= 1 << signal;
}

uint8_t readAndClearSignal(uint8_t signal) {
	if ((uint32_icc_signals & (1 << signal)) > 0) {
		uint32_icc_signals &= ~(1 << signal);
		return 1;
	}
	return 0;
}

void initErpsRatio(void) {
	//if (readAndClearSignal(SIGNAL_SPEEDLIMIT_CHANGED) == 1) 
	ui16_speed_kph_to_erps_ratio = (uint16_t) ((float) ui8_gear_ratio * 1000000.0 / ((float) wheel_circumference * 36.0));
}

void updateHallOrder(uint8_t hall_sensors) {
	
	if (++ui8_hall_order_counter > 5) {
		ui8_hall_order_counter = 0;
	}
	uint8_t_hall_order[ui8_hall_order_counter] = hall_sensors;
	current_hall = hall_sensors;
}

void updatePasDir(void) {
	if (((ui8_aca_flags_high & TQ_SENSOR_MODE) == TQ_SENSOR_MODE)&&(ui16_time_ticks_between_pas_interrupt < timeout)) {
		//only for Torquesensor Mode.
		PAS_is_active = 1;
	} else if (((ui8_aca_flags_high & TQ_SENSOR_MODE) != TQ_SENSOR_MODE) && (PAS_act > 3)) {
		//set direction only if enough pulses in the right direction are detected.
		PAS_is_active = 1;
	} else {
		PAS_is_active = 0;
	}
}

void updateX4(void) {
	ui16_x4_value = ui16_adc_read_x4_value();
}

/*
 * Generated with https://www.sebulli.com/ntc/index.php?lang=en&points=256&unit=1&resolution=10+Bit&circuit=pullup&resistor=100000&r25=100000&beta=3480&test_resistance=100000&tmin=0&tmax=75
 */
static const int16_t NTC_table[257] = {
  351, 294, 237, 208, 189, 176, 165, 156, 149,
  143, 138, 133, 129, 125, 121, 118, 115, 112,
  110, 107, 105, 103, 101, 99, 97, 95, 93,
  92, 90, 89, 87, 86, 85, 83, 82, 81, 80, 79,
  77, 76, 75, 74, 73, 72, 71, 70, 70, 69, 68,
  67, 66, 65, 65, 64, 63, 62, 61, 61, 60, 59,
  59, 58, 57, 57, 56, 55, 55, 54, 53, 53, 52,
  52, 51, 50, 50, 49, 49, 48, 48, 47, 47, 46,
  46, 45, 45, 44, 43, 43, 42, 42, 42, 41, 41,
  40, 40, 39, 39, 38, 38, 37, 37, 36, 36, 35,
  35, 35, 34, 34, 33, 33, 32, 32, 32, 31, 31,
  30, 30, 29, 29, 29, 28, 28, 27, 27, 27, 26,
  26, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22,
  21, 21, 21, 20, 20, 19, 19, 19, 18, 18, 18,
  17, 17, 16, 16, 16, 15, 15, 14, 14, 14, 13,
  13, 12, 12, 12, 11, 11, 11, 10, 10, 9, 9,
  9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 4, 3,
  3, 2, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -2,
  -3, -3, -4, -4, -5, -5, -6, -6, -7, -7, -8,
  -8, -9, -9, -10, -10, -11, -12, -12, -13,
  -13, -14, -14, -15, -16, -16, -17, -18, -18,
  -19, -20, -20, -21, -22, -23, -24, -24, -25,
  -26, -27, -28, -29, -30, -31, -32, -34, -35,
  -36, -38, -39, -41, -43, -45, -47, -50, -53,
  -57, -62, -71, -80
};

int8_t x4_ntc_value(void) {
    int16_t p1,p2;
    p1 = NTC_table[ (ui16_x4_value >> 2)  ];
    p2 = NTC_table[ (ui16_x4_value >> 2)+1];
      /* Interpolate between both points. */
    return p1 - ( (p1-p2) * (ui16_x4_value & 0x0003) ) / 4;
}

//added by DerBastler - Light		
void updateLight(void) {
	if (((light_stat &1) != 1)&& ((light_stat &128) == 128)) {
		light_pin_set();
		light_stat = (light_stat&~1) | 1;
	}else if (((light_stat &1) == 1)&& ((light_stat &128)!= 128)) {
		light_pin_reset();
		light_stat = (light_stat&~1) | 0;
	}
}


void updateRequestedTorque(void) {
    ui8_momentary_throttle = (uint16_t) map8u(ui8_adc_read_throttle(), ADC_THROTTLE_MIN_VALUE, ADC_THROTTLE_MAX_VALUE, 0, SETPOINT_MAX_VALUE);
	if (((ui8_aca_flags_high & TQ_SENSOR_MODE) != TQ_SENSOR_MODE)) {
		ui16_throttle_accumulated -= ui16_throttle_accumulated >> 4;
		ui16_throttle_accumulated += ui8_adc_read_throttle();
		ui8_temp = ui16_throttle_accumulated >> 4; //read in value from adc
		ui16_sum_throttle = (uint8_t) map8u(ui8_temp, ui8_throttle_min_range, ui8_throttle_max_range, 0, SETPOINT_MAX_VALUE); //map throttle to limits
	} else {

		ui16_sum_torque = 0;
		for (ui8_temp = 0; ui8_temp < NUMBER_OF_PAS_MAGS; ui8_temp++) { // sum up array content
			ui16_sum_torque += ui16_torque[ui8_temp];
		}

		ui16_sum_torque /= NUMBER_OF_PAS_MAGS;

	}

	//For cruise behaviour -> cruise is disabled on these conditions
	if (ui8_cruiseThrottleSetting > 0) {
		if (ui16_sum_throttle > ui8_cruiseMinThrottle + 5) {
			ui8_cruiseThrottleSetting = 0;
		}
		if (ui16_sum_throttle < ui8_cruiseMinThrottle) {
			ui8_cruiseMinThrottle = ui16_sum_throttle;
		}
	}
}

void checkPasInActivity(void) {
	ui8_PAS_update_call_when_inactive_counter++;
	if (ui16_time_ticks_for_pas_calculation > timeout) {
		// updatePasStatus does not fire if pas inactive, so set interval to reasonably high value here
		ui16_time_ticks_between_pas_interrupt = 64000L;
		// also ensure torque array slowly resets
		ui16_torque[ui8_torque_index] = (uint8_t) map8u(ui8_throttle_min_range, ui8_throttle_min_range, ui8_throttle_max_range, 0, SETPOINT_MAX_VALUE); //map throttle to limits
		ui8_torque_index++;
		if (ui8_torque_index > NUMBER_OF_PAS_MAGS - 1) {
			ui8_torque_index = 0;
		}
		
	}
	// we are called at 50 Hz, if there has been no interrupt for more than ~1s, ramp down PAS automatically
	if (ui8_PAS_Flag == 0 && ui8_PAS_update_call_when_inactive_counter > (uint8_t) (timeout >> 6)) {

		ui8_PAS_update_call_when_inactive_counter = 0;

		if (PAS_act > 0) {
			PAS_act--;
		}

		updatePasDir();

	}
}

void updatePasStatus(void) {

	if (ui8_PAS_Flag == 1) {
		ui8_PAS_Flag = 0; //reset interrupt flag

		ui16_time_ticks_between_pas_interrupt = ui16_time_ticks_for_pas_calculation; //save recent cadence
		ui16_PAS_High = ui16_PAS_High_Counter;

		if ((0 == (ui8_aca_flags_low & PAS_INVERTED)) && ((float) ui16_time_ticks_between_pas_interrupt / (float) ui16_PAS_High > flt_s_pas_threshold)) {
			if (PAS_act < 7) {
				PAS_act++;
			}
		} else if ((PAS_INVERTED == (ui8_aca_flags_low & PAS_INVERTED)) && ((float) ui16_time_ticks_between_pas_interrupt / (float) ui16_PAS_High < flt_s_pas_threshold)) {
			if (PAS_act < 7) {
				PAS_act++;
			}
		} else {
			if (PAS_act > 0) {
				PAS_act--;
			}
		}


		if (((ui8_aca_flags_high & TQ_SENSOR_MODE) == TQ_SENSOR_MODE)) {
			ui16_temp = ui16_adc_read_x4_value(); //read in recent torque value
			ui16_torque[ui8_torque_index] = (uint8_t) map32(ui16_temp, ui8_throttle_min_range, ui8_throttle_max_range, 0, SETPOINT_MAX_VALUE); //map throttle to limits

			ui8_torque_index++;
			if (ui8_torque_index > NUMBER_OF_PAS_MAGS - 1) {
				ui8_torque_index = 0;
			} //reset index counter

		}

		updatePasDir();
		ui16_time_ticks_for_pas_calculation = 1;
		ui16_PAS_High_Counter = 1; //reset PAS Counter
		ui8_PAS_update_call_when_inactive_counter = 0;
	}
}

void updateSlowLoopStates(void) {
	
	if (ui16_no_pass_counter < 64000){
		ui16_no_pass_counter++;
	}
	
	if (ui16_motor_speed_erps == 0) {
		if (ui16_idle_counter < 64000){
			ui16_idle_counter++;
		}
	} else {
		ui16_idle_counter = 0;
	}
	
	// debug only
	//ui8_variableDebugC = ui16_no_pass_counter>>8;
	
	//disable lock if passcode is not at least 4 digits
	if (ui16_passcode < 1001){
		ui8_lockstatus = 16;
		ui16_no_pass_counter =0;
	}else if (ui16_no_pass_counter > 3000) {
		//lock after 60 seconds idle
		ui8_lockstatus = 255;
	}
	
	if (((ui8_aca_flags_low & IDLE_DISABLES_OFFROAD) == IDLE_DISABLES_OFFROAD) && (ui8_offroad_state > 4) && (ui16_idle_counter > 3000)) {
		//disable after 60 seconds idle
		ui8_offroad_state = 0;
	}
	
	if (((ui8_aca_flags_low & BRAKE_DISABLES_OFFROAD) == BRAKE_DISABLES_OFFROAD) && (ui8_offroad_state > 4)) {
		// if disabling is enabled :)
		if (!GPIO_ReadInputPin(BRAKE__PORT, BRAKE__PIN)) {
			ui8_offroad_counter++;
			if (ui8_offroad_counter == 255) {//disable on pressing brake for 5 seconds
				ui8_offroad_state = 0;
				ui8_offroad_counter = 0;
			}
		} else {
			ui8_offroad_counter = 0;
		}
	}

	// check if offroad mode is enabled
	if (0 == (ui8_aca_flags_low & OFFROAD_ENABLED)) {
		return;
	}

	if (ui8_offroad_state == 0 && !GPIO_ReadInputPin(BRAKE__PORT, BRAKE__PIN)) {//first step, brake on.
		ui8_offroad_state = 1;
	} else if (ui8_offroad_state == 1) {//second step, make sure the brake is hold according to definded time
		ui8_offroad_counter++;
		if (GPIO_ReadInputPin(BRAKE__PORT, BRAKE__PIN) && ui8_offroad_counter < MORSE_TIME_1) {//brake is released too early
			ui8_offroad_state = 0;
			ui8_offroad_counter = 0;
		} else if (GPIO_ReadInputPin(BRAKE__PORT, BRAKE__PIN) && ui8_offroad_counter > MORSE_TIME_1 + MORSE_TOLERANCE) {//brake is released according to cheatcode
			ui8_offroad_state = 2;
			ui8_offroad_counter = 0;
		} else if (!GPIO_ReadInputPin(BRAKE__PORT, BRAKE__PIN) && ui8_offroad_counter > MORSE_TIME_1 + MORSE_TOLERANCE) {//brake is released too late
			ui8_offroad_state = 0;
			ui8_offroad_counter = 0;
		}
	} else if (ui8_offroad_state == 2) {//third step, make sure the brake is released according to definded time
		ui8_offroad_counter++;
		if (!GPIO_ReadInputPin(BRAKE__PORT, BRAKE__PIN) && ui8_offroad_counter < MORSE_TIME_2) { //brake is hold too early
			ui8_offroad_state = 0;
			ui8_offroad_counter = 0;
		} else if (!GPIO_ReadInputPin(BRAKE__PORT, BRAKE__PIN) && ui8_offroad_counter > MORSE_TIME_2 + MORSE_TOLERANCE) {//brake is hold according to cheatcode
			ui8_offroad_state = 3;
			ui8_offroad_counter = 0;

		} else if (GPIO_ReadInputPin(BRAKE__PORT, BRAKE__PIN) && ui8_offroad_counter > MORSE_TIME_2 + MORSE_TOLERANCE) {//brake is hold too late
			ui8_offroad_state = 0;
			ui8_offroad_counter = 0;
		}
	} else if (ui8_offroad_state == 3) {//second step, make sure the brake is hold according to definded time
		ui8_offroad_counter++;
		if (GPIO_ReadInputPin(BRAKE__PORT, BRAKE__PIN) && ui8_offroad_counter < MORSE_TIME_3) {//brake is released too early
			ui8_offroad_state = 0;
			ui8_offroad_counter = 0;
		} else if (GPIO_ReadInputPin(BRAKE__PORT, BRAKE__PIN) && ui8_offroad_counter > MORSE_TIME_3 + MORSE_TOLERANCE) {//brake is released according to cheatcode
			ui8_offroad_state = 4;
			ui8_offroad_counter = 0;

		} else if (!GPIO_ReadInputPin(BRAKE__PORT, BRAKE__PIN) && ui8_offroad_counter > MORSE_TIME_3 + MORSE_TOLERANCE) {//brake is released too late
			ui8_offroad_state = 0;
			ui8_offroad_counter = 0;
		}
	} else if (ui8_offroad_state == 4) {
		// wait 3 seconds in state 4 for display feedback
		ui8_offroad_counter++;
		if (ui8_offroad_counter > 150) {
			ui8_offroad_state = 255;
			ui8_offroad_counter = 0;
		}
	}


}
