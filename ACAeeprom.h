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
 * File:   ACAeeprom.h
 * Author: Björn Schmidt
 *
 * Created on August 30, 2018, 8:03 PM
 */

#ifndef BOEEPROM_H
#define BOEEPROM_H

#include "config.h"

extern uint8_t eeprom_magic_byte;

#define EEPROM_BASE_ADDRESS    0x4000
#define EEPROM_MAX_INIT_RANGE 0x1F // we should at least have 640 avail / 32 in use for now

typedef enum
{
    OFFSET_MOTOR_ANGLE = ((uint8_t) 0x01),
	OFFSET_THROTTLE_REACTS_TO_ASSIST_LEVEL = ((uint8_t) 0x02),
    OFFSET_ASSIST_LEVEL = ((uint8_t) 0x03),
    OFFSET_THROTTLE_MIN_RANGE = ((uint8_t) 0x04),
    OFFSET_THROTTLE_MAX_RANGE = ((uint8_t) 0x05),
    OFFSET_PAS_DIRECTION = ((uint8_t) 0x06),
    OFFSET_PAS_TRESHOLD = ((uint8_t) 0x07),
    OFFSET_PID_GAIN_P = ((uint8_t) 0x08),
    OFFSET_PID_GAIN_I = ((uint8_t) 0x09),
    OFFSET_RAMP_END = ((uint8_t) 0x0A),
			
	OFFSET_MAX_SPEED_DEFAULT = ((uint8_t) 0x0B),
	OFFSET_MAX_SPEED_WITHOUT_PAS = ((uint8_t) 0x0C),
	OFFSET_MAX_SPEED_WITH_THROTTLE_OVERRIDE = ((uint8_t) 0x0D)

} BO_EEPROM_OFFSETS;

void eeprom_init(void);
uint8_t eeprom_write(uint8_t adress_offset, uint8_t value);
uint8_t eeprom_read(uint8_t address_offset);

#endif /* BOEEPROM_H */