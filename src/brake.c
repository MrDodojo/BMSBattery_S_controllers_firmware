/*
 * EGG OpenSource EBike firmware
 *
 * Copyright (C) Casainho, 2015, 2106, 2017.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include "stm8s.h"
#include "stm8s_it.h"
#include "gpio.h"
#include "main.h"
#include "interrupts.h"
#include "brake.h"
#include "cruise_control.h"
#include "motor.h"
#include "pwm.h"

// Brake signal
void EXTI_PORTA_IRQHandler(void) __interrupt(EXTI_PORTA_IRQHANDLER)
{
    // original code did nothing except kill the mcu when brake was enabled at startup (will never retire)
    return;
}

void brake_init (void)
{
  //hall sensors pins as external input pin interrupt
  GPIO_Init(BRAKE__PORT,
	    BRAKE__PIN,
#ifdef BRAKE_NO_INTERRUPT
        GPIO_MODE_IN_FL_NO_IT
#else
	    GPIO_MODE_IN_FL_IT // with external interrupt
#endif
        );

  //initialize the Interrupt sensitivity
#ifndef BRAKE_NO_INTERRUPT
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOA, EXTI_SENSITIVITY_RISE_FALL);
#endif
}

BitStatus brake_is_set (void)
{
  if (GPIO_ReadInputPin(BRAKE__PORT, BRAKE__PIN) == 0)
    return 1;
  else
    return 0;
}

void brake_coast_enable (void)
{
  TIM1->BKR &= (uint8_t) ~(TIM1_BKR_MOE);
}

void brake_coast_disable (void)
{
  TIM1->BKR |= (uint8_t) (TIM1_BKR_MOE);
}
