/*
Generic display init and update functions
Written by Thomas Jarosch

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
#ifndef DISPLAY_H
#define DISPLAY_H

#include "config.h"

void display_init();
void display_update();
//void display_debug(HardwareSerial* localSerial);

//void display_show_important_info(const char *str, int duration_secs);
//void display_show_important_info(const __FlashStringHelper *str, int duration_secs);

void display_show_welcome_msg();
void display_show_welcome_msg_temp();

void display_prev_view();
void display_next_view();



//definitions for different screen mode
typedef enum {DISPLAY_MODE_TEXT,
              DISPLAY_MODE_GRAPHIC,               // Note: Same as _TEXT on 16x2 displays
              DISPLAY_MODE_MENU,
              DISPLAY_MODE_IMPORTANT_INFO
             } display_mode_type;

typedef enum {DISPLAY_VIEW_MAIN=0,

              _DISPLAY_VIEW_END
             } display_view_type;

extern display_view_type display_view;

extern display_mode_type display_mode; //currently display mode
extern uint8_t display_force_text;         //only valid for Nokia displays
extern uint16_t poti_stat;
extern uint16_t throttle_stat;
extern uint8_t battery_percent_fromvoltage;
extern uint8_t battery_percent_fromcapacity;
extern uint32_t wheel_time;
extern float current_display;


#ifdef DISPLAY_TYPE_KT_LCD3
typedef struct _lcd_configuration_variables
{
  uint8_t ui8_light_On;        //added by DerBasteler	
  uint8_t ui8_WalkModus_On;    //added by DerBasteler	
  uint8_t ui8_assist_level;
  uint8_t ui8_max_speed;
  uint8_t ui8_wheel_size;
  uint8_t ui8_p1;
  uint8_t ui8_p2;
  uint8_t ui8_p3;
  uint8_t ui8_p4;
  uint8_t ui8_p5;
  uint8_t ui8_c1;
  uint8_t ui8_c2;
  uint8_t ui8_c4;
  uint8_t ui8_c5;
  uint8_t ui8_c12;
  uint8_t ui8_c13;
  uint8_t ui8_c14;
} struc_lcd_configuration_variables;
#elif defined DISPLAY_TYPE_KT_LCD8

typedef struct LCD8_display_data_t {
	union  {
		uint8_t raw[13];
		struct {
			uint8_t 	p5; // B0
			uint8_t 	assist_level : 3;
			uint8_t 	unknown : 4;
			uint8_t 	lights : 1; // B1
			uint8_t 	wheel_size_msb : 3;
			uint8_t 	max_speed_lsb : 5; // B2
			uint8_t 	p1; // B3
			uint8_t 	p2 : 3;
			uint8_t 	p3 : 1;
			uint8_t 	p4 : 1;
			uint8_t 	max_speed_msb : 1;
			uint8_t 	wheel_size_lsb : 2; // B4
			uint8_t 	crc; // B5
			uint8_t 	c2 : 3;
			uint8_t		c1 : 3;
			uint8_t 	unkown2 : 2; // B6
			uint8_t 	c5 : 4;
			uint8_t 	unknown4 : 1;
			uint8_t 	c14 : 2;
			uint8_t 	unknown3 : 1; // B7
			uint8_t 	c12 : 5;
			uint8_t 	c4 : 3; // B8
			uint8_t 	B9; // B9
			uint8_t 	unknown6 : 2;
			uint8_t 	c13 : 3;
			uint8_t 	c15 : 2;
			uint8_t 	unknown5 : 1; // B10
			uint8_t 	c4_percentage : 6;
			uint8_t 	l1 : 2; // B11
			uint8_t 	B12;
		};
	};
	uint8_t max_speed;
	uint8_t wheel_size;
} LCD8_display_data;

typedef struct LCD8_controller_data_t {
	union {
		uint8_t raw[12];
		struct {
			uint8_t B0; // always 0x41
			uint8_t charging_status : 2;
			uint8_t bars : 3;
			uint8_t unknown1 : 3; // B1
			uint8_t nominal_voltage : 6;
			uint8_t reverse : 1;
			uint8_t unknown2 : 1; // B2
			uint16_t rotation; // B3+B4
			uint8_t error; // B5
			uint8_t crc; // B6
			uint8_t mode_normal : 1;
			uint8_t mode_throttle : 1;
			uint8_t mode_unknown : 1;
			uint8_t mode_cruise : 1;
			uint8_t mode_assist : 1;
			uint8_t mode_regen : 1;
			uint8_t unknown4 : 2; // B7
			uint8_t amps; // B8
			uint8_t motor_temperature; // B9
			uint8_t B10;
			uint8_t B11;
			uint8_t B12;
		};
	};
} LCD8_controller_data;
#endif

#endif
