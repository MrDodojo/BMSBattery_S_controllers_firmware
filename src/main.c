/*
 * BMSBattery S series motor controllers firmware
 *
 * Copyright (C) Casainho, 2017.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>
#include "stm8s.h"
#include "gpio.h"
#include "stm8s_itc.h"
#include "stm8s_gpio.h"
#include "interrupts.h"
#include "stm8s_tim2.h"
#include "motor.h"
#include "main.h"
#include "uart.h"
#include "adc.h"
#include "brake.h"
#include "cruise_control.h"
#include "timers.h"
#include "pwm.h"
#include "PAS.h"
#include "SPEED.h"
//#include "update_setpoint.h"
#include "ACAsetPoint.h"
#include "config.h"
#include "display.h"
#include "display_kingmeter.h"
#include "ACAcontrollerState.h"
#include "BOdisplay.h"
#include "ACAeeprom.h"
#include "ACAcommons.h"

//uint16_t ui16_LPF_angle_adjust = 0;
//uint16_t ui16_LPF_angle_adjust_temp = 0;
extern int8_t hall_sensors;
uint16_t ui16_log1 = 0;
uint8_t ui8_slowloop_flag = 0;
uint8_t ui8_veryslowloop_counter = 0;
uint8_t ui8_ultraslowloop_counter = 0;
uint16_t ui16_log2 = 0;
uint8_t ui8_log = 0;
uint8_t ui8_i = 0; //counter for ... next loop

float float_kv = 0;
float float_R = 0;
uint8_t a = 0; //loop counter

static int16_t i16_deziAmps;


/////////////////////////////////////////////////////////////////////////////////////////////
//// Functions prototypes

// main -- start of firmware and main loop
int main(void);

//With SDCC, interrupt service routine function prototypes must be placed in the file that contains main ()
//in order for an vector for the interrupt to be placed in the the interrupt vector space.  It's acceptable
//to place the function prototype in a header file as long as the header file is included in the file that
//contains main ().  SDCC will not generate any warnings or errors if this is not done, but the vector will
//not be in place so the ISR will not be executed when the interrupt occurs.

//Calling a function from interrupt not always works, SDCC manual says to avoid it. Maybe the best is to put
//all the code inside the interrupt

// Local VS global variables
// Sometimes I got the following error when compiling the firmware: motor.asm:750: Error: <r> relocation error
// and the solution was to avoid using local variables and define them as global instead

// Brake signal interrupt
void EXTI_PORTA_IRQHandler(void) __interrupt(EXTI_PORTA_IRQHANDLER);
// Speed signal interrupt
void EXTI_PORTC_IRQHandler(void) __interrupt(EXTI_PORTC_IRQHANDLER);
// PAS signal interrupt
void EXTI_PORTD_IRQHandler(void) __interrupt(EXTI_PORTD_IRQHANDLER);

// Timer1/PWM period interrupt
void TIM1_UPD_OVF_TRG_BRK_IRQHandler(void) __interrupt(TIM1_UPD_OVF_TRG_BRK_IRQHANDLER);

// Timer2/slow control loop
void TIM2_UPD_OVF_TRG_BRK_IRQHandler(void) __interrupt(TIM2_UPD_OVF_TRG_BRK_IRQHANDLER);

// UART2 receivce handler
void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER);


/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

int main(void) {
	//set clock at the max 16MHz
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);

	gpio_init();
	brake_init();
	while (brake_is_set()); // hold here while brake is pressed -- this is a protection for development
	debug_pin_init();
	light_pin_init();
	light_pin_reset(); //seemed to fix floating issues
	timer2_init();
	uart_init();
	eeprom_init();
	controllerstate_init();
	initErpsRatio();
	pwm_init();
	hall_sensor_init();
	adc_init();
	PAS_init();
	SPEED_init();
	aca_setpoint_init();
#if (defined (DISPLAY_TYPE) && defined (DISPLAY_TYPE_KINGMETER)) || defined DISPLAY_TYPE_KT_LCD3 || defined BLUOSEC || defined DISPLAY_TYPE_LCD8
	display_init();
#endif

	//  ITC_SetSoftwarePriority (ITC_IRQ_TIM1_OVF, ITC_PRIORITYLEVEL_2);

	enableInterrupts();

	watchdog_init(); //init watchdog after enabling interrupt to have fast loop running already

    debug_pin_set();
#if (SVM_TABLE == SVM)
	TIM1_SetCompare1(126 << 1);
	TIM1_SetCompare2(126 << 1);
	TIM1_SetCompare3(126 << 1);
#elif (SVM_TABLE == SINE) || (SVM_TABLE == SINE_SVM)
	TIM1_SetCompare1(126 << 2);
	TIM1_SetCompare2(126 << 2);
	TIM1_SetCompare3(126 << 2);
#endif

	hall_sensors_read_and_action(); // needed to start the motor

	for (a = 0; a < NUMBER_OF_PAS_MAGS; a++) {// array init
		ui16_torque[a] = 0;
	}
    debug_pin_reset();

#if defined TT
	ui8_aca_experimental_flags_low |= DC_STATIC_ZERO;// | PWM_AUTO_OFF;
    /* ui8_aca_experimental_flags_high = USE_ALTERNATE_WAVETABLE | USE_ALTERNATE_WAVETABLE_C; */
    ui8_aca_experimental_flags_low  = 0;
    /* ui8_aca_flags_high = 0; */
    /* ui8_aca_flags_low = 0; */
#endif

#if defined DIAGNOSTICS

	printf("System initialized\r\n");

#endif

    uint8_t pts = 0;
	while (1) {
		uart_send_if_avail();
		updateSpeeds();
		updatePasStatus(); // 5us if idle

#if (defined (DISPLAY_TYPE) && defined (DISPLAY_TYPE_KINGMETER)) || defined DISPLAY_TYPE_KT_LCD3 || defined BLUOSEC || defined DISPLAY_TYPE_KT_LCD8

		display_update(); // 54us
#endif

		// scheduled update of setpoint and duty cycle (slow loop, 50 Hz)
        // slow loop in default operation takes about 1.2ms instead of about 5-6
		if (ui8_slowloop_flag) {
            
		    debug_pin_set();
			ui8_slowloop_flag = 0; //reset flag for slow loop
			ui8_veryslowloop_counter++; // increase counter for very slow loop
            motor_slow_update_pre(); // 2us
			checkPasInActivity(); // 12us
			updateRequestedTorque(); // 75us
			updateSlowLoopStates(); // 8us
#ifndef X4_TEMPERATURE
			updateX4();
#endif
			updateLight(); // 2us
                           //
			ui16_setpoint = (uint16_t) aca_setpoint(ui16_time_ticks_between_pas_interrupt, ui16_setpoint);  // used to b 4000+us, now about 720us

/* #if DO_CRUISE_CONTROL == 1 */
/* 			ui16_setpoint = cruise_control(ui16_setpoint); */
/* #endif */

			pwm_set_duty_cycle((uint8_t) ui16_setpoint); // 3us 
            motor_slow_update_post(); // 4us
			/****************************************************************************/
			//very slow loop for communication
			if (ui8_veryslowloop_counter > 5) {

				ui8_ultraslowloop_counter++;
				ui8_veryslowloop_counter = 0;

				if (ui8_ultraslowloop_counter > 10) {
					ui8_ultraslowloop_counter = 0;
					ui8_uptime++;
#ifdef X4_TEMPERATURE
			        updateX4();// update+ntc onv takes 61us
                    i8_motor_temperature = x4_ntc_value(); // low side ntc, use iff you have a 100k pullup and 100k ntc
					//i8_motor_temperature = (ui16_x4_value - 105) >> 1;
					//i8_motor_temperature += (int8_t)(ui16_BatteryCurrent - ui16_current_cal_b - 2) / 9; //if temperature sensor is not in the exact same common ground point you can add some compensation according to battery current
#endif
					//*/
				}

#ifdef DIAGNOSTICS
				//printf("%u,%u, %u, %u, %u, %u\r\n", ui16_control_state, ui16_setpoint, ui16_motor_speed_erps, ui16_BatteryCurrent, ui16_sum_torque, ui8_momentary_throttle);

				//printf("erps %d, motorstate %d, cyclecountertotal %d\r\n", ui16_motor_speed_erps, ui8_possible_motor_state|ui8_dynamic_motor_state, ui16_PWM_cycles_counter_total);

				//printf("cheatstate, %d, km/h %lu, Voltage, %d, setpoint %d, erps %d, current %d, correction_value, %d\n", ui8_offroad_state, ui32_speed_sensor_rpks, ui8_BatteryVoltage, ui16_setpoint, ui16_motor_speed_erps, ui16_BatteryCurrent, ui8_position_correction_value);

				//printf("kv %d, erps %d, R %d\n", (uint16_t)(float_kv*10.0) , ui16_motor_speed_erps, (uint16_t)(float_R*1000.0));

				/*for(a = 0; a < 6; a++) {			// sum up array content
						 putchar(uint8_t_hall_case[a]);
						 }
				putchar(ui16_ADC_iq_current>>2);
				putchar(ui8_position_correction_value);
				putchar(255);*/
				// printf("%d, %d, %d, %d, %d, %d\r\n", (uint16_t) uint8_t_hall_case[0], (uint16_t)uint8_t_hall_case[1],(uint16_t) uint8_t_hall_case[2],(uint16_t) uint8_t_hall_case[3], (uint16_t)uint8_t_hall_case[4], (uint16_t)uint8_t_hall_case[5]);
				printf("%d, %d, %d, %d, %d, %d, %d,\r\n", ui8_position_correction_value, ui16_BatteryCurrent, ui16_setpoint, ui16_motor_speed_erps, ui16_ADC_iq_current>>2,ui16_adc_read_battery_voltage());


				printf("correction angle %d, Current %d, Voltage %d, sumtorque %d, setpoint %d, km/h %lu\n",ui8_position_correction_value, i16_deziAmps, ui8_BatteryVoltage, ui16_sum_throttle, ui16_setpoint, ui32_speed_sensor_rpks);
#endif
			}//end of very slow loop


        debug_pin_reset();
		}// end of slow loop

#ifdef TT
        pts++;
               if (pts >= 25) {
                       pts = 0;
                       uint8_t crc = 4; // randomly chosen with a true die
                        putchar(0xAA);

                        crc ^= 0xAA;
                        volatile uint16_t uc = ui16_adc_read_phase_B_current();
                        putchar(uc >> 8);
                        putchar(uc  & 0xFF);
                        crc ^= uc >> 8;
                        crc ^= (uc & 0xFF);
                        putchar(current_hall);
                        crc ^= current_hall;
                        putchar(crc);
                        //uart_send_if_avail();
               }
#endif

	}// end of while(1) loop
}


