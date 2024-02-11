/*
 * EGG OpenSource EBike firmware
 *
 * Copyright (C) Casainho, 2015, 2106, 2017.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _PWM_H
#define _PWM_H

#include "main.h"
#include "interrupts.h"
#include "config.h"

#define SVM 1
#define SINE_SVM_ORIGINAL 2
#define SINE 3

#define SVM_TABLE SVM

#ifndef SWAP_PHASES
#define SWAP_PHASES_DEFAULT 2
#else
#define SWAP_PHASES_DEFAULT SWAP_PHASES
#endif


extern uint8_t ui8_duty_cycle_target;

void pwm_init (void);
void pwm_duty_cycle_controller (void);
void pwm_set_duty_cycle (uint8_t value);
void pwm_apply_duty_cycle (uint8_t ui8_duty_cycle_value);

#endif /* _PWM_H_ */
