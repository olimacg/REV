/****************************************************************************
 * @file     isr_TimerD.c
 * @brief    Timer D Interrupt Service Routine
 * @date     22 September 2015
 *
 * @note
 * Copyright (C) 2017-2019, Qorvo, Inc.
 *
 * THIS SOFTWARE IS SUBJECT TO A SOURCE CODE LICENSE AGREEMENT WHICH PROVIDES,
 * AMONG OTHER THINGS:  (i) THAT IT CAN BE USED ONLY TO ADAPT THE LICENSEE'S
 * APPLICATION TO PAC PROCESSORS SUPPLIED BY QORVO, INC.;
 * (ii) THAT  IT IS PROVIDED "AS IS" WITHOUT WARRANTY;  (iii) THAT
 * QORVO, INC. IS NOT LIABLE FOR ANY INDIRECT DAMAGES OR FOR DIRECT
 * DAMAGES EXCEEDING US$1,500;  AND (iv) THAT IT CAN BE DISCLOSED TO AND USED
 * ONLY BY CERTAIN AUTHORIZED PERSONS.
 ******************************************************************************/
#define INCLUDE_EXTERNS
#include "bldc_common.h"

#if defined SENSORLESS_APP || defined RC_PWM_THROTTLE_APP

/**
 * @brief	This is the interrupt handler for Timer D, which gets serviced 30 degrees after zero crossing.
 *			When this interrupt is serviced, it will set the current state to the next state.  The current
 *			state needs to change to the next state 30 degrees after the zero crossing has been detected.
 *
 * @return none
 *
 */
APP_RAMFUNC void TimerD_IRQHandler(void)
{
	if (PAC55XX_TIMERD->INT.CCR0IF)
		{
		PAC55XX_TIMERD->CCTL0.CCINTEN = 0;
		PAC55XX_GPIOB->OUT.w = c_pwm_io_state[motordir][sl_current_state];
#ifdef PAC5556
		PAC55XX_SCC->PBMUXSEL.w = psel_mask_pbmux[motordir][sl_current_state];	        // Set peripheral select state
		PAC55XX_SCC->PCMUXSEL.w = psel_mask_pcmux[motordir][sl_current_state];	        // Set peripheral select state
#else
		PAC55XX_SCC->PBMUXSEL.w = psel_mask[motordir][sl_current_state];	        // Set peripheral select state
#endif

		// Switch mux to the sensorlesss comparator that needs to be monitored during the next state
		pac5xxx_tile_register_write(ADDR_CFGAIO9, slcomp_mux[motordir][sl_current_state]);

		sl_current_state++;
		if (sl_current_state > 5)
			sl_current_state = 0;

		open_loop = 0;
		getslcomp_state = TimerC_getslcomp_blanking_cycles;
		PAC55XX_TIMERC->INT.CCR0IF = 1;
		PAC55XX_TIMERC->CCTL0.CCINTEN = 1;
		PAC55XX_TIMERD->INT.CCR0IF = 1;
		}

	if (PAC55XX_TIMERD->INT.BASEIF)
		{
		switch (TMRD_State)
			{
			case TimerD_Idle:
				//Do Nothing
			break;

			case TimerD_SineWaveOL:
				if (motordir)
					{
					sine_index += 1;
					if (sine_index >= 360)
						sine_index = 0;
					}
				else
					{
					sine_index -= 1;
					if (sine_index >= 0xFFFE)
						sine_index = 359;
					}

				fix16_t temp0, temp1, temp2;

				temp0 = fix16_mul_new_16_16(sine_wave_3phase[sine_index][0], sine_scale);
				temp1 = fix16_mul_new_16_16(sine_wave_3phase[sine_index][1], sine_scale);
				temp2 = fix16_mul_new_16_16(sine_wave_3phase[sine_index][2], sine_scale);

				if (temp0 == 0) temp0++;
				if (temp1 == 0) temp1++;
				if (temp2 == 0) temp2++;

				PAC55XX_TIMER_SEL->CCTR4.CTR = temp0;
				PAC55XX_TIMER_SEL->CCTR5.CTR = temp1;
				PAC55XX_TIMER_SEL->CCTR6.CTR = temp2;

				sl_current_state = fix16_mul_new_16_16(sine_index, ONEDIV60);
				break;

			case TimerD_SixStepOL:
				PAC55XX_GPIOB->OUT.w = c_pwm_io_state[motordir][sl_current_state];
#ifdef PAC5556

				PAC55XX_SCC->PBMUXSEL.w = psel_mask_pbmux[motordir][sl_current_state];	        // Set peripheral select state
				PAC55XX_SCC->PCMUXSEL.w = psel_mask_pcmux[motordir][sl_current_state];	        // Set peripheral select state
#else
				PAC55XX_SCC->PBMUXSEL.w = psel_mask[motordir][sl_current_state];	        // Set peripheral select state
#endif
				// Switch mux to the sensorlesss comparator that needs to be monitored during the next state
				pac5xxx_tile_register_write(ADDR_CFGAIO9, slcomp_mux[motordir][sl_current_state]);

				sl_current_state++;
				if (sl_current_state > 5)
					sl_current_state = 0;
				getslcomp_state = TimerC_getslcomp_blanking_cycles;
				break;

			case TimerD_Switchover:
				PAC55XX_GPIOB->OUT.w = c_pwm_io_state[motordir][sl_current_state];
#ifdef PAC5556
				PAC55XX_SCC->PBMUXSEL.w = psel_mask_pbmux[motordir][sl_current_state];	        // Set peripheral select state
				PAC55XX_SCC->PCMUXSEL.w = psel_mask_pcmux[motordir][sl_current_state];	        // Set peripheral select state
#else
				PAC55XX_SCC->PBMUXSEL.w = psel_mask[motordir][sl_current_state];	        // Set peripheral select state
#endif

				// Switch mux to the sensorlesss comparator that needs to be monitored during the next state
				pac5xxx_tile_register_write(ADDR_CFGAIO9, slcomp_mux[motordir][sl_current_state]);

				sl_current_state++;
				if (sl_current_state > 5)
					sl_current_state = 0;
				getslcomp_state = TimerC_getslcomp_blanking_cycles;

				PAC55XX_TIMERD->PRD.w = 0xFFFF;
	        	tmp_blanking_cycles = blanking_cycles;
				PAC55XX_TIMERD->INT.CCR0IF = 1;						//Clear TIMER D CCR0 Flag
				PAC55XX_TIMERD->CCTL0.CCINTEN = 0;					//Disable TIMER D CCR0 Interrupt
				PAC55XX_TIMERC->INT.CCR0IF = 1;						//Clear TIMER C CCR0 Flag
	        	PAC55XX_TIMERC->CCTL0.CCINTEN = 1;					//Enable TIMER C CCR0 ISR
				NVIC_EnableIRQ(TimerC_IRQn);
				blanking_cycles = RUN_BLANKING_CYCLES;
				good_samples = RUN_GOOD_SAMPLES;
				open_loop = 0;
				app_status |= status_closed_loop;
				break;
			}
		PAC55XX_TIMERD->INT.BASEIF = 1;


		}
}

#endif

#ifdef HALL_SENSOR_APP

/**
 * @brief  This is the interrupt handler for Timer B
 *
 * @return none
 *
 */
APP_RAMFUNC void TimerD_IRQHandler(void)
{
	if (PAC55XX_TIMERD->INT.BASEIF)
		{
#if ANGLE_ADVANCE
	//Commutation Engine for Angle Advanced Based Hall Sensor
		if ((app_status & status_motor_enabled) && (next_commutation_state != 0xFF))
			{
			PAC55XX_SCC->PBMUXSEL.w = psel_mask[motordir][next_commutation_state];	        // Set peripheral select state
			PAC55XX_GPIOB->OUT.w = c_pwm_io_state[motordir][next_commutation_state];
			}
		PAC55XX_TIMERD->CTL.BASEIE = 0;
#endif
		PAC55XX_TIMERD->INT.BASEIF = 1;
		}
	if (PAC55XX_TIMERD->INT.CCR0IF)
		{
		PAC55XX_TIMERD->INT.CCR0IF = 1;
		}
	if (PAC55XX_TIMERD->INT.CCR1IF)
		{
		PAC55XX_TIMERD->INT.CCR1IF = 1;
		}
	if (PAC55XX_TIMERD->INT.CCR2IF)
		{
		PAC55XX_TIMERD->INT.CCR2IF = 1;
		}
	if (PAC55XX_TIMERD->INT.CCR3IF)
		{
		PAC55XX_TIMERD->INT.CCR3IF = 1;
		}
	if (PAC55XX_TIMERD->INT.CCR4IF)
		{
		PAC55XX_TIMERD->INT.CCR4IF = 1;
		}
	if (PAC55XX_TIMERD->INT.CCR5IF)
		{
		PAC55XX_TIMERD->INT.CCR5IF = 1;
		}
	if (PAC55XX_TIMERD->INT.CCR6IF)
		{
		PAC55XX_TIMERD->INT.CCR6IF = 1;
		}
	if (PAC55XX_TIMERD->INT.CCR7IF)
		{
		PAC55XX_TIMERD->INT.CCR7IF = 1;
		}
}

#endif
