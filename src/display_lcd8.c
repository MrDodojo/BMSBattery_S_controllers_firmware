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
#include <string.h>
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
#include "pwm.h"

//#define DEBUG

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

void send_message(void) {

	// prepare moving indication info
	
	
	//if (ebike_app_cruise_control_is_set ()) { ui8_moving_indication |= (1 << 3); }
	//if (throttle_is_set ()) { ui8_moving_indication |= (1 << 1); }
	//if (pas_is_set ()) { ui8_moving_indication |= (1 << 4); }


	if (((ui8_aca_flags_high & EXTERNAL_SPEED_SENSOR) == EXTERNAL_SPEED_SENSOR)) {
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
	ui16_battery_bars_calc = ui8_adc_read_battery_voltage();
    if (ui16_battery_bars_calc > BATTERY_VOLTAGE_MAX_VALUE - BATTERY_CHARGING_VS_FULL_OFFSET) {
        ui16_battery_bars_calc = 1024;
    } else if (ui16_battery_bars_calc < BATTERY_VOLTAGE_MIN_VALUE) {
        ui16_battery_bars_calc = 0;
    } else {
        ui16_battery_bars_calc -= BATTERY_VOLTAGE_MIN_VALUE;
        ui16_battery_bars_calc<<=8;
        ui16_battery_bars_calc /=(BATTERY_VOLTAGE_MAX_VALUE - BATTERY_VOLTAGE_MIN_VALUE);
    }
#ifdef DEBUG
    memset(&controller_data, 0x00, sizeof(controller_data));
#endif
    if (ui16_battery_bars_calc == 1024) {
        controller_data.charging_status = 2;
		controller_data.bars = 0;
    } // charging
    else	if (ui16_battery_bars_calc > 204) {
        controller_data.charging_status = 0;
		controller_data.bars = 4;
	}// 4 bars | full
	else if (ui16_battery_bars_calc > 154) {
        controller_data.charging_status = 0;
		controller_data.bars = 3;
	}// 3 bars
	else if (ui16_battery_bars_calc > 102) {
        controller_data.charging_status = 0;
		controller_data.bars = 2;
	}// 2 bars
	else if (ui16_battery_bars_calc > 51) {
        controller_data.charging_status = 0;
		controller_data.bars = 1;
	}// 1 bar
	else if (ui16_battery_bars_calc > 25) {
        controller_data.charging_status = 0;
        controller_data.bars = 0;
	} // empty
    else {
        controller_data.charging_status = 1;
        controller_data.bars = 0;
    } // flashing
      //
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
	controller_data.mode_brake = MVI(5);
	controller_data.mode_assist = MVI(4);
	controller_data.mode_cruise = MVI(3);
    controller_data.mode_cruise_icon = MVI(3); // cruise set?
	controller_data.mode_throttle = MVI(1);
	controller_data.mode_normal = MVI(0);
	// B8: 4x controller current
	// Vbat = 30V:
	// - B8 = 255, LCD shows 1912 watts
	// - B8 = 250, LCD shows 1875 watts
	// - B8 = 100, LCD shows 750 watts
	// each unit of B8 = 0.25A
	

	/* if (ui16_BatteryCurrent <= ui16_current_cal_b) { //avoid full power displayed at regen and avoid small watts being displayed when the bike  */
	/* 	controller_data.amps = 0; */
	/* } */
	/* else { */
    if (ui16_BatteryCurrent < ui16_current_cal_b) {
    	controller_data.amps = ui16_current_cal_b - ui16_BatteryCurrent; 
        controller_data.mode_brake = 1;
    } else {
    	controller_data.amps = (uint8_t)((((ui16_BatteryCurrent - ui16_current_cal_b - 1) << 2) * 10) / ui8_current_cal_a);
    }

	// B9: motor temperature
    controller_data.motor_temperature = i8_motor_temperature - 15; //according to documentation at endless sphere
	// B10 and B11: 0
	controller_data.B10 = 0x00;
	controller_data.B11 = 0x00;
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
	light_stat = (light_stat&~128) | (lcd_data.lights << 7); // only update 7th bit, 1st bit is current status

	/* if (lcd_data.max_speed != ui8_speedlimit_kph) { */
    /*     putchar(lcd_data.max_speed); */
    /*     putchar(ui8_speedlimit_kph); */
	/* 	ui8_speedlimit_kph = lcd_data.max_speed; */
	/* 	eeprom_write(OFFSET_MAX_SPEED_DEFAULT, lcd_data.max_speed); */
	/* } */
    uint8_t update_aca = 0;
    uint8_t update_exp = 0;

	ui8_s_motor_angle = lcd_data.p1;	
	
    if (pwm_swap_phases != lcd_data.p2) {
        pwm_swap_phases = lcd_data.p2; // alows live swapping of 
                                       // phases to test motors
        pwm_duty_cycle_controller();
    }

	if (lcd_data.p3 != (ui8_aca_experimental_flags_high >> 1) & 0x01) {
		ui8_aca_experimental_flags_high &= ~PWM_AUTO_OFF;
		ui8_aca_experimental_flags_high |= lcd_data.p3 << 1;
        update_exp = 1;
	}

	if (lcd_data.p4 != (ui8_aca_flags_low >> 7)) {
		ui8_aca_flags_low &= ~PAS_INVERTED;
		ui8_aca_flags_low |= lcd_data.p4 << 7;
        update_aca = 1;
	}
	
    if (lcd_data.c1 != ((ui8_aca_flags_low >> 1) & 0x07)) {
		ui8_aca_flags_low &= ~(OFFROAD_ENABLED | BRAKE_DISABLES_OFFROAD | IDLE_DISABLES_OFFROAD);
        ui8_aca_flags_low |= lcd_data.c1 << 1;
        update_aca = 1;
    }

	if (lcd_data.c2 != (ui8_aca_flags_low & 0x01)) {
		ui8_aca_flags_low &= ~ASSIST_LVL_AFFECTS_THROTTLE;
		ui8_aca_flags_low |= lcd_data.c2;
        update_aca = 1;
	}

    
    if (lcd_data.c5 != ((ui8_aca_experimental_flags_low >> 5) & 0x07)) {
		ui8_aca_experimental_flags_low &= ~(USE_ALTERNATE_WAVETABLE | USE_ALTERNATE_WAVETABLE_B | USE_ALTERNATE_WAVETABLE_C);
        ui8_aca_experimental_flags_low |= (lcd_data.c5 & 0x07) << 5;
        update_exp = 1;
    }

    if (lcd_data.c12 != ((ui8_aca_flags_low >> 4) & 0x07)) {
		ui8_aca_flags_low &= ~(DIGITAL_REGEN | SPEED_INFLUENCES_REGEN | SPEED_INFLUENCES_TORQUESENSOR);
        ui8_aca_flags_low |= lcd_data.c12 << 4;
        update_aca = 1;
    }

    if (lcd_data.l1 != ((ui8_aca_experimental_flags_high >> 3) & 0x03)) {
        ui8_aca_experimental_flags_high &= ~(THROTTLE_ALLOWED_FOR_WALK |  THROTTLE_REGEN | THROTTLE_UNRESTRICTED);
        switch (lcd_data.l1) {
            case 0:
            default:
                ui8_aca_experimental_flags_high |= THROTTLE_ALLOWED_FOR_WALK;
                break;
            case 1:
                ui8_aca_experimental_flags_high |= THROTTLE_REGEN;
                break;

            case 2:
                ui8_aca_experimental_flags_high |= THROTTLE_UNRESTRICTED;
                break;
            case 3:
                ui8_aca_experimental_flags_high |= THROTTLE_UNRESTRICTED | THROTTLE_REGEN;
                break;
        }
        update_exp = 1;
    }

	if (lcd_data.l2 != ((ui8_aca_experimental_flags_low >> 1) & 0x01)) {
		ui8_aca_experimental_flags_low &= ~AVOID_MOTOR_CYCLES_JITTER;
		ui8_aca_experimental_flags_low |= lcd_data.l2 << 1;
        update_exp = 1;
	}

	if (lcd_data.l3 != ((ui8_aca_flags_high >> 2) & 0x01)) {
		ui8_aca_flags_high &= ~DYNAMIC_ASSIST_LEVEL;
		ui8_aca_flags_high |= lcd_data.l3 << 2;
        update_aca = 1;
	}

    if (update_aca) {
        debug_pin_set();
		eeprom_write(OFFSET_ACA_FLAGS_HIGH_BYTE, ui8_aca_flags_high);
		eeprom_write(OFFSET_ACA_FLAGS, ui8_aca_flags_low);
        debug_pin_reset();
    }

    if (update_exp) {
        debug_pin_set();
		eeprom_write(OFFSET_ACA_EXPERIMENTAL_FLAGS_HIGH_BYTE, ui8_aca_experimental_flags_high);
		eeprom_write(OFFSET_ACA_EXPERIMENTAL_FLAGS, ui8_aca_experimental_flags_low);
        debug_pin_reset();
    }
}

// see if we have a received package to be processed

void display_update(void) {

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
			lcd_data.wheel_size = lcd_data.wheel_size_msb << 1 | lcd_data.wheel_size_lsb;
            /* wheel sizes:
             * 0 = 16"
             * 1 = 18"
             * 2 = 20"
             * 3 = 23"
             * 4 = 24"
             * 5 = 26"
             * 6 = 27.5"
             * 7 = 28"
             * 8 = 12"
             * 9 = 14"
             * 10 = 8"
             * 11 = 10"
             * 12 = 6"
             * 13 = 5"
             * 14 = not available
             * 15 = 29"
             */

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
