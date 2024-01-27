/*
Generic display init and update functions
Written by jenkie and Thomas Jarosch
Functions for the Nokia graphical screen mainly by m--k
King-Meter library and support written by Michael Fabry

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software Foundation,
Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
 */

#include <stdio.h>
#include "stm8s.h"
#include "display.h"
#include "main.h"
#include "config.h"
#include "stm8s_itc.h"
#include "uart.h"
#include "adc.h"
#include "brake.h"
#include "ACAeeprom.h"
#include "interrupts.h"
#include "ACAcontrollerState.h"

extern uint8_t pwm_swap_phases;
#ifdef DISPLAY_TYPE_KT_LCD8
display_view_type display_view;
display_mode_type display_mode; //currently display mode
float current_display;
uint8_t battery_percent_fromcapacity = 11; //hier nur als Konstante um Batterie normal zu senden....


uint8_t ui8_j;
uint8_t ui8_crc;
uint16_t ui16_wheel_period_ms = 4500;
uint16_t ui16_battery_bars_calc = 0;
uint8_t ui8_battery_soc = 12;
uint8_t ui16_error;
uint8_t ui8_rx_buffer_counter = 0;
uint8_t ui8_byte_received;

uint8_t ui8_UARTCounter = 0;
uint8_t ui8_cruiseHasBeenLow;

volatile LCD8_display_data lcd_data;
LCD8_controller_data controller_data;

void display_init(){
	// noop just here to have a common interface
}

void send_message() {

	// prepare moving indication info
	
	
	//if (ebike_app_cruise_control_is_set ()) { ui8_moving_indication |= (1 << 3); }
	//if (throttle_is_set ()) { ui8_moving_indication |= (1 << 1); }
	//if (pas_is_set ()) { ui8_moving_indication |= (1 << 4); }


	if (((ui16_aca_flags & EXTERNAL_SPEED_SENSOR) == EXTERNAL_SPEED_SENSOR)) {
		if (ui16_time_ticks_between_speed_interrupt > 65000) {
			ui16_wheel_period_ms = 4500;
		} else {
			ui16_wheel_period_ms = (uint16_t) ((float) ui16_time_ticks_between_speed_interrupt / ((float) ui16_pwm_cycles_second / 1000.0)); //must be /1000 devided in /125/8 for better resolution
		}
	}else{
		if (ui32_erps_filtered == 0) {
			ui16_wheel_period_ms = 4500;
		} else {
			ui16_wheel_period_ms = (uint16_t) (1000.0 * (float) ui8_gear_ratio / (float) ui32_erps_filtered);
		}
	}

	// calc battery pack state of charge (SOC)
	ui16_battery_bars_calc = ui8_adc_read_battery_voltage() - ui8_s_battery_voltage_min;
	ui16_battery_bars_calc<<=8;
	ui16_battery_bars_calc /=(ui8_s_battery_voltage_max-ui8_s_battery_voltage_min);

	controller_data.charging_status = 0;
	if (ui16_battery_bars_calc > 200) {
		controller_data.bars = 4;
	}// 4 bars | full
	else if (ui16_battery_bars_calc > 150) {
		controller_data.bars = 3;
	}// 3 bars
	else if (ui16_battery_bars_calc > 100) {
		controller_data.bars = 2;
	}// 2 bars
	else if (ui16_battery_bars_calc > 50) {
		controller_data.bars = 1;
	}// 1 bar
	else {
		controller_data.charging_status = 3;
	} // empty

	controller_data.B0 = 0x41;
	// B2: 24V controller
	controller_data.nominal_voltage = ui8_battery_voltage_nominal;
	// B3: speed, wheel rotation period, ms; period(ms)=B3*256+B4;
	controller_data.rotation = ui16_wheel_period_ms; // + 0x1B0A;

	//Send confirming signal for activating offroad mode
	if (ui8_offroad_state == 4) { //quitting signal for offroad mode enabled. Shows about 80 km/h for three seconds

		controller_data.rotation = 0xFFFF; // will always result in 80kph
	}

	// B5: error info display
	controller_data.error = ui16_error;
	// B6: CRC: xor B1,B2,B3,B4,B5,B7,B8,B9,B10,B11
	// 0 value so no effect on xor operation for now
	// B7: moving mode indication, bit
#define MVI(x) ((ui8_moving_indication >> x) & 0x01)
	controller_data.mode_regen = MVI(5);
	controller_data.mode_assist = MVI(4);
	controller_data.mode_cruise = MVI(3);
	controller_data.mode_throttle = MVI(1);
	controller_data.mode_normal = MVI(0);
	// B8: 4x controller current
	// Vbat = 30V:
	// - B8 = 255, LCD shows 1912 watts
	// - B8 = 250, LCD shows 1875 watts
	// - B8 = 100, LCD shows 750 watts
	// each unit of B8 = 0.25A
	

	if (ui16_BatteryCurrent <= ui16_current_cal_b + 2) { //avoid full power displayed at regen and avoid small watts being displayed when the bike 
		controller_data.amps = 0;
	}
	else {
		controller_data.amps = (uint8_t)((((ui16_BatteryCurrent - ui16_current_cal_b - 1) << 2) * 10) / ui8_current_cal_a);
	}
	// B9: motor temperature
	controller_data.motor_temperature = i8_motor_temperature - 15; //according to documentation at endless sphere
	// B10 and B11: 0
	controller_data.B10 = 0;
	controller_data.B11 = 0;
	//for (int i = 1; i < 12; i++) controller_data.raw[i] = 0xaa;
	// calculate CRC xor
	controller_data.crc = 0;
	ui8_crc = 0;
	for (ui8_j = 1; ui8_j <= 11; ui8_j++) {
		ui8_crc ^= controller_data.raw[ui8_j];
	}
	controller_data.crc = ui8_crc;


	// send the package over UART
	for (ui8_j = 0; ui8_j <= 11; ui8_j++) {
		uart_put_buffered(controller_data.raw[ui8_j]);
		//putchar(controller_data.raw[ui8_j]);
	}
}

/********************************************************************************************/
// Process received package from the LCD
//

void digestLcdValues(void) {

	ui8_assistlevel_global = lcd_data.assist_level + 80; // always add max regen 
	ui8_walk_assist = (lcd_data.assist_level == 6);
	// added by DerBastler Light On/Off
	light_stat = (light_stat&~128) | lcd_data.lights; // only update 7th bit, 1st bit is current status
	
	if (lcd_data.max_speed != ui8_speedlimit_kph) {
		ui8_speedlimit_kph = lcd_data.max_speed;
		eeprom_write(OFFSET_MAX_SPEED_DEFAULT, lcd_data.max_speed);
	}

/*	
typedef enum {

	DC_STATIC_ZERO = ((uint16_t) 1),
	AVOID_MOTOR_CYCLES_JITTER = ((uint16_t) 2),
	DISABLE_INTERPOLATION = ((uint16_t) 4),
	DISABLE_60_DEG_INTERPOLATION = ((uint16_t) 8),
	SWITCH_360_DEG_INTERPOLATION = ((uint16_t) 16),
	USE_ALTERNATE_WAVETABLE = ((uint16_t) 32),
	USE_ALTERNATE_WAVETABLE_B = ((uint16_t) 64),
	DUMMY_EXP_ALWAYS_ON = ((uint16_t) 128),
	HIGH_SPEED_MOTOR = ((uint16_t) 256),
	PWM_AUTO_OFF = ((uint16_t) 1024),
			
} ACA_EXPERIMENTAL_FLAGS;*/
    uint16_t old_experimental = ui16_aca_experimental_flags;
    uint16_t old_aca = ui16_aca_flags;

	ui8_s_motor_angle = lcd_data.p1;	
	
	if (lcd_data.c14) { // can be >1 but all set value
		ui16_aca_experimental_flags |= AVOID_MOTOR_CYCLES_JITTER;
	} else {
		ui16_aca_experimental_flags &= ~AVOID_MOTOR_CYCLES_JITTER;
	}

	if (lcd_data.p3) {
		ui16_aca_experimental_flags |= PWM_AUTO_OFF;
	} else {
		ui16_aca_experimental_flags &= ~PWM_AUTO_OFF;
	}

	if (lcd_data.p4) {
		ui16_aca_experimental_flags |= DC_STATIC_ZERO;
	} else {
		ui16_aca_experimental_flags &= ~DC_STATIC_ZERO;
	}

	if (lcd_data.c1 & 0x01) {
		ui16_aca_flags |= OFFROAD_ENABLED;
	} else {
		ui16_aca_flags &= ~OFFROAD_ENABLED;
	}

	if (lcd_data.c1 & 0x02) {
		ui16_aca_flags |= BRAKE_DISABLES_OFFROAD;
	} else {
		ui16_aca_flags &= ~BRAKE_DISABLES_OFFROAD;
	}

	if (lcd_data.c1 & 0x04) {
		ui16_aca_flags |= IDLE_DISABLES_OFFROAD;
	} else {
		ui16_aca_flags &= ~IDLE_DISABLES_OFFROAD;
	}

	if (lcd_data.c2) {
		ui16_aca_flags |= PAS_INVERTED;
	} else {
		ui16_aca_flags &= ~PAS_INVERTED;
	}

	if (lcd_data.c4) {
		ui16_aca_flags |= ASSIST_LVL_AFFECTS_THROTTLE;
	} else {
		ui16_aca_flags &= ~ASSIST_LVL_AFFECTS_THROTTLE;
	}

    pwm_swap_phases = lcd_data.c14; // alows live swapping of 
                                   // phases to test motors
    if (ui16_aca_flags != old_aca) {
		eeprom_write(OFFSET_ACA_FLAGS_HIGH_BYTE, (ui16_aca_flags >> 8)& 0xFF);
		eeprom_write(OFFSET_ACA_FLAGS, ui16_aca_flags & 0xFF);
    }

    if (ui16_aca_experimental_flags != old_experimental) {
		eeprom_write(OFFSET_ACA_EXPERIMENTAL_FLAGS_HIGH_BYTE, (ui16_aca_experimental_flags >> 8)& 0xFF);
		eeprom_write(OFFSET_ACA_EXPERIMENTAL_FLAGS, ui16_aca_experimental_flags & 0xFF);
    }
}

// see if we have a received package to be processed

void display_update() {

	// fill local buffer from uart ringbuffer
	// new: we put in a packed union so we do not have to mess with calculating the offset
	uart_fill_rx_packet_buffer(lcd_data.raw, 13, &ui8_UARTCounter);
	
	// Check for reception of complete message
	if ((ui8_UARTCounter > 12)) { // && (lcd_data.B12 == 0x0E)) {
		ui8_UARTCounter = 0;

		// validation of the package data
		ui8_crc = 0;
		for (ui8_j = 0; ui8_j <= 12; ui8_j++) {
			
			if (ui8_j == 5) continue; // don't xor B5 
			ui8_crc ^= lcd_data.raw[ui8_j];
		}

		// see if CRC is ok
		if (((ui8_crc ^ 10) == lcd_data.crc) || // some versions of CRC LCD5 (??)
                                ((ui8_crc ^ 1) == lcd_data.crc) || // CRC LCD3 (tested with KT36/48SVPR, from PSWpower)
                                ((ui8_crc ^ 2) == lcd_data.crc) || // CRC LCD5
                                ((ui8_crc ^ 3) == lcd_data.crc) || // CRC LCD5 Added display 5 Romanta
                                ((ui8_crc ^ 4) == lcd_data.crc) ||
                                ((ui8_crc ^ 5) == lcd_data.crc) ||
                                ((ui8_crc ^ 6) == lcd_data.crc) ||
                                ((ui8_crc ^ 7) == lcd_data.crc) ||
                                ((ui8_crc ^ 8) == lcd_data.crc) ||
                                ((ui8_crc ^ 14) == lcd_data.crc) ||
                                ((ui8_crc ^ 9) == lcd_data.crc) ||// CRC LCD3
                                ((ui8_crc ^ 23) == lcd_data.crc)) // CRC of an LCD8
		{
			// added by DerBastler Light On/Off 
			// walk assist, see https://endless-sphere.com/forums/viewtopic.php?f=2&t=73471&p=1324745&hilit=kunteng+protocol+hacked#p1109048 
			lcd_data.max_speed = 10 + (lcd_data.max_speed_msb << 3 | lcd_data.max_speed_lsb);
			lcd_data.wheel_size = lcd_data.wheel_size_msb << 2 | lcd_data.wheel_size_lsb;

			if (lcd_data.B9 == 0) {
				ui8_cruiseHasBeenLow = 1;
			}
			if (lcd_data.assist_level != (ui8_assistlevel_global & 15)) { //cruise disables when switching assist levels, just like stock
				ui8_cruiseThrottleSetting = 0;
			}
			else if (lcd_data.c12 == 16 && lcd_data.assist_level != 0 && ui8_cruiseThrottleSetting == 0 && ui8_cruiseHasBeenLow == 1) {
				ui8_cruiseHasBeenLow = 0;
				ui8_cruiseThrottleSetting = ui16_sum_throttle;
				ui8_cruiseMinThrottle = ui8_cruiseThrottleSetting;
			}

			digestLcdValues();
			send_message();
		}
	}
}

#endif
