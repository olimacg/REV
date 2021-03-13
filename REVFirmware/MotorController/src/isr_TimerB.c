/****************************************************************************
 * @file     isr_TimerB.c
 * @brief    Timer B Interrupt Service Routine
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

#if defined SENSORLESS_APP || defined HALL_SENSOR_APP

/**
 * @brief  This is the interrupt handler for Timer B
 *
 * @return none
 *
 */
APP_RAMFUNC void TimerB_IRQHandler(void)
{
	if (PAC55XX_TIMERB->INT.BASEIF)
		{
		PAC55XX_TIMERB->INT.BASEIF = 1;
		}
	if (PAC55XX_TIMERB->INT.CCR0IF)
		{
		PAC55XX_TIMERB->INT.CCR0IF = 1;
		}
	if (PAC55XX_TIMERB->INT.CCR1IF)
		{
		PAC55XX_TIMERB->INT.CCR1IF = 1;
		}
	if (PAC55XX_TIMERB->INT.CCR2IF)
		{
		PAC55XX_TIMERB->INT.CCR2IF = 1;
		}
	if (PAC55XX_TIMERB->INT.CCR3IF)
		{
		PAC55XX_TIMERB->INT.CCR3IF = 1;
		}
	if (PAC55XX_TIMERB->INT.CCR4IF)
		{
		PAC55XX_TIMERB->INT.CCR4IF = 1;
		}
	if (PAC55XX_TIMERB->INT.CCR5IF)
		{
		PAC55XX_TIMERB->INT.CCR5IF = 1;
		}
	if (PAC55XX_TIMERB->INT.CCR6IF)
		{
		PAC55XX_TIMERB->INT.CCR6IF = 1;
		}
	if (PAC55XX_TIMERB->INT.CCR7IF)
		{
		PAC55XX_TIMERB->INT.CCR7IF = 1;
		}
}
#endif

#ifdef RC_PWM_THROTTLE_APP

/**
 * @brief  This is the interrupt handler for Timer B.
 * It is in charge of reading the RC Input on PD2 and extracting RC information
 *
 * @return none
 *
 */
APP_RAMFUNC void TimerB_IRQHandler(void)
{

	uint32_t tempus_interrupt = PAC55XX_TIMERB->INT.w;

	if (PAC55XX_TIMERB->INT.BASEIF) //pac5xxx_timer_b_base_int()
		{
		if (good_pulse)
			{
			good_pulse = 0;
			}
		else
			{
			rcpwm_period_counter--;
			if (rcpwm_period_counter <= 0)
				{
				rcpwm_period_counter = 0;
				rc_status = RCGone;
				if (app_status & status_motor_enabled)
					{
					motor_pwm_disable();
					}
				SMS_State = SMS_Idle;
				}
			}
		PAC55XX_TIMERB->INT.BASEIF = 1;
		}

	if (PAC55XX_TIMERB->INT.CCR7IF)
			{
			//PWMB7 Interrupt Requests will be issued by both rise and falling edge triggering points.
			//We need to determine whether a rising edge or a falling edge has occurred.
			//By looking at the digital state of the PC7 pin, we can determine whether a rising or
			//a falling edge has been registered. If HI, it must have been a rising and vice versa.

			//We will use rising edges to measure RC PWM period and determine whether the transmitting
			//signal is present. The rising edge timing information will also be used to measure pulse width.
			//We will use falling edges to measure the RC PWM pulse width and compute motor PWM duty cycle.

			if (RC_PWM_IN_GPIO & (1 << 7))			//rising edge detected
				{
				legal_fedge = 1;

				prev_rcpwm_time_rise = rcpwm_time_rise;
				rcpwm_time_rise = PAC55XX_TIMERB->CCTR7.w;

				if (rcpwm_time_rise > prev_rcpwm_time_rise)			// No overflow occurred
					{
					rcpwm_period = rcpwm_time_rise - prev_rcpwm_time_rise;
					}
				else												//Overflow occurred
					{
					rcpwm_period = (0xFFFF - prev_rcpwm_time_rise) + rcpwm_time_rise;
					}

				//RC pulses should be spaced about 22 ms apart when an RC radio is present.
				//When the RC radio is not present, RC PWM pulses may still be present, although
				//they are of a spurious nature. We want to ensure that only legal pulses are
				//processed and conducive of motor PWM output.
				//The rcpwm_peiod ensures that the time between rising edges is in the 22 ms vicinity.
				//If not, the pulse is spurious. 200 ms worth of spurious activity will cause the
				//system to be disabled.
				//Once disabled, the system can only return to enablement if a number of consecutive
				//and valid pulses equal to MAX_PWM_PERIOD_COUNTS are found.
				if ((rcpwm_period > RCPWM_PERIOD_LO) && (rcpwm_period < RCPWM_PERIOD_HI))
					{
					rcpwm_period_counter++;
					good_pulse = 1;
					if (rcpwm_period_counter > MAX_PWM_PERIOD_COUNTS)
						{
						rcpwm_period_counter = MAX_PWM_PERIOD_COUNTS;
						if (rc_status == RCGone)
							{
							rc_status = RCPresent;
							SMS_State = SMS_Wait_Radio_Command;
							}
						}
					}
				else
					{
					rcpwm_period_counter--;
					good_pulse = 0;
					if (rcpwm_period_counter <= 0)
						{
						rcpwm_period_counter = 0;
						rc_status = RCGone;
						if (app_status & status_motor_enabled)
							{
							motor_pwm_disable();
							}
						SMS_State = SMS_Idle;
						}
					}
				}
			else									//falling edge detected
				{
				if (legal_fedge)
					{
					legal_fedge = 0;
					rcpwm_time_fall = PAC55XX_TIMERB->CCTR7.w;

					if (rcpwm_time_fall > rcpwm_time_rise)			// No overflow occurred
						{
						rcpwm_width = rcpwm_time_fall - rcpwm_time_rise;
						}
					else												//Overflow occurred
						{
						rcpwm_width = (0xFFFF - rcpwm_time_rise) + rcpwm_time_fall;
						}

					rc_width_ms = rcpwm_width << 16;
					rc_width_ms = fix16_mul(rc_width_ms, FREQ_INV);

					//We need to extract the RC Command
					//The RC Command will be a Motor PWM output ranging from 0% to 100% duty cycle
					//A reading of 1.0 ms or less (1563 ticks) will command 0% duty cycle
					//A reading of 2.0 ms or more (3125 ticks) will command 100% duty cycle
					if (rcpwm_width < RCPWM_WIDTH_MIN)
						{
						rcpwm_width = RCPWM_WIDTH_MIN;
						}
					else if (rcpwm_width > RCPWM_WIDTH_MAX)
						{
						rcpwm_width = RCPWM_WIDTH_MAX;
						}

		#if FULL_THROTTLE_RC

						//Any reading between 1.0 ms and 2.0 ms, will command a proportional speed in Hz between 0 Hz and RC_MAX_SPEED_HZ Hz.
						//The RC Command is a number from 1563 ticks to 3125 ticks normalized to 1.
						//We subtract the minimum PWM WIDTH from the sampled number of ticks. 1 ms becomes 0, 2 ms becomes 1.
						//We multiply by the MAX_HZ_SPEED and divide by the maximum number of timer B ticks.
						//The RCPWM_NORMALIZE_CONSTANT = RC_MAX_SPEED_HZ / RCPWM_WIDTH_MIN
						rc_command = (rcpwm_width - RCPWM_WIDTH_MIN) << 16;
						rc_command = fix16_mul(rc_command, RCPWM_NORMALIZE_CONSTANT);
						speed_ref_command_hz = rc_command >> 16;

						//Any reading between 1.0 ms and 2.0 ms, will command a proportional duty cycle 300 ticks and 2450 ticks.
						//The RC Command is a number from 1563 ticks to 3125 ticks normalized to 1.
						//We subtract the minimum PWM WIDTH from the sampled number of ticks. 1 ms becomes 0, 2 ms becomes 1.
						//We multiply by the MAX_DUTY_CYCLE and divide by the maximum number of timer B ticks.
						//The RCPWM_NORMALIZE_DC_CONSTANT = MAX_DUTY_CYCLE / RCPWM_WIDTH_MIN
						rc_command = (rcpwm_width - RCPWM_WIDTH_MIN) << 16;
						rc_command = fix16_mul(rc_command, RCPWM_NORMALIZE_DC_CONSTANT);
						mtrpwm_width = rc_command >> 16;

		#else
						//Any reading between 1.5 ms and 2.0 ms, will command a proportional speed in Hz between 0 Hz and RC_MAX_SPEED_HZ Hz.
						//Any reading between 1.0 ms and 1.5 ms, will command a proportional speed in Hz between RC_MAX_SPEED_HZ Hz and 0 Hz.
						//The RC Command is a number from 1563 ticks to 3125 ticks normalized to 1.
						//For Forward, we subtract the RCPWM_WIDTH_MID from the sampled number of ticks.
						//For Reverse, we subtract the sampled number of ticks from RCPWM_WIDTH_MID.
						//For Forwards, 1.5 ms becomes 0 and 2 ms becomes 1.
						//For Reverse, 1.5 ms becomes 0 and 0 ms becomes 1.
						//We multiply by the MAX_HZ_SPEED and divide by the maximum number of timer B ticks.
						//The RCPWM_NORMALIZE_CONSTANT = RC_MAX_SPEED_HZ / RCPWM_WIDTH_MIN

						if (rcpwm_width > RCPWM_WIDTH_MID)
							{
							rc_command = (rcpwm_width - RCPWM_WIDTH_MID) << 16;
							command_mdir = 1;
							}
						else
							{
							rc_command = (RCPWM_WIDTH_MID - rcpwm_width) << 16;
							command_mdir = 0;
							}
						rc_command = fix16_mul(rc_command, RCPWM_NORMALIZE_CONSTANT);
						speed_ref_command_hz = rc_command >> 16;
						mtrpwm_width = rc_command >> 16;
		#endif
					}
				}
			PAC55XX_TIMERB->INT.CCR7IF = 1;
			}
}
#endif
