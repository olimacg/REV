/****************************************************************************
 * @file     isr_TimerD.c
 * @brief    Timer D Interrupt Service Routine
 * @date     22 September 2015
 *
 * @note
 * Copyright (C) 2015-2019, Qorvo
 *
 * THIS SOFTWARE IS SUBJECT TO A SOURCE CODE LICENSE AGREEMENT WHICH PROVIDES,
 * AMONG OTHER THINGS:  (i) THAT IT CAN BE USED ONLY TO ADAPT THE LICENSEE'S
 * APPLICATION TO PAC PROCESSORS SUPPLIED BY QORVO;
 * (ii) THAT  IT IS PROVIDED "AS IS" WITHOUT WARRANTY;  (iii) THAT
 * QORVO, INC. IS NOT LIABLE FOR ANY INDIRECT DAMAGES OR FOR DIRECT
 * DAMAGES EXCEEDING US$1,500;  AND (iv) THAT IT CAN BE DISCLOSED TO AND USED
 * ONLY BY CERTAIN AUTHORIZED PERSONS.
 ******************************************************************************/
#define INCLUDE_EXTERNS
#include "bldc_common.h"

#ifdef RC_PWM_THROTTLE_APP
/**
 * @brief  The state machine is in charge of coordination motor operation. Interval is 1 ms.
 *
 * @return none
 *
 */
void state_machine(void)
{

	switch (SMS_State)
		{
		case SMS_Idle:
			if (rc_status)
				{
				SMS_State = SMS_Wait_Radio_Command;
				}

			break;
		case SMS_Wait_Radio_Command:
			if (!(app_status & status_motor_enabled) && (speed_ref_command_hz >= RC_SPEED_TH_HZ_ON) && !(app_status & status_under_voltage))
					{
					motordir = command_mdir;
					if (motordir)
						{
						app_status |= status_motor_direction;
						}
					else
						{
						app_status &= ~status_motor_direction;
						}

					SMS_State = SMS_Align_6S;
					}
			break;

		case SMS_Align_6S:
			ADCSM_State = ADCSM_IRegulate;
			app_status &= ~status_motor_stalled;
			sl_current_state = 0;

			//tmp_enable_current_pi = 0;

			if (enable_current_pi && enable_speed_pi) //Current and Speed PI Modes Enabled
				{
				ol_iqref_increase = fix16_div(start_iq_ref << 16, align_time_ms << 16);
				iq_ref = ol_iqref_increase;
				iq_pid.min_value = dt_leading_ticks << 16;
				iq_pid.PI_sat = iq_pid.min_value;
				iq_pid.I_prev = iq_pid.min_value;
				speed_pid.PI_sat = iq_ref;
				speed_pid.I_prev = iq_ref;
				pwm_duty = iq_pid.min_value >> 16;
				}
			else if (!enable_current_pi && enable_speed_pi) //Speed PI Mode Only
				{
				ol_iqref_increase = fix16_div(pwm_duty << 16, align_time_ms << 16);
				pwm_duty = DT_LED_TICKS + ol_iqref_increase >> 16;
				speed_pwm_pid.PI_sat = pwm_duty;
				speed_pwm_pid.I_prev = pwm_duty;
				}
			else if (enable_current_pi && !enable_speed_pi) //Current PI Only
				{
				ol_iqref_increase = fix16_div(start_iq_ref << 16, align_time_ms << 16);
				iq_ref = ol_iqref_increase;
				iq_pid.min_value = dt_leading_ticks << 16;
				iq_pid.PI_sat = iq_pid.min_value;
				iq_pid.I_prev = iq_pid.min_value;
				speed_pid.PI_sat = iq_ref;
				speed_pid.I_prev = iq_ref;
				pwm_duty = DT_LED_TICKS + iq_pid.min_value >> 16;
				}
			else 											//PWM Mode
				{
				ol_iqref_increase = fix16_div(pwm_duty << 16, align_time_ms << 16);
				pwm_duty = DT_LED_TICKS + (ol_iqref_increase >> 16);
				}

#ifdef PAC5556
			PAC55XX_SCC->PBMUXSEL.w = psel_mask_pbmux[motordir][sl_current_state];	        // Set peripheral select state
			PAC55XX_SCC->PCMUXSEL.w = psel_mask_pcmux[motordir][sl_current_state];	        // Set peripheral select state
#else
			PAC55XX_SCC->PBMUXSEL.w = psel_mask[motordir][sl_current_state];	        // Set peripheral select state
#endif
			PAC55XX_GPIOB->OUT.w = c_pwm_io_state[motordir][sl_current_state];

			blanking_cycles = START_BLANKING_CYCLES;
			good_samples = START_GOOD_SAMPLES;

			SMS_Counter = 0;
			SMS_State = SMS_Align_Wait_6S;
			app_status |= status_motor_enabled;
			open_loop = 1;
		break;

		case SMS_Align_Wait_6S:
			SMS_Counter ++;

			if (SMS_Counter >= align_time_ms)
				{
				SMS_Counter = 0;
				SMS_State = SMS_Start_Motor_6S;
				tmp_enable_current_pi = enable_current_pi;
				}
			else
				{
				if (enable_current_pi && enable_speed_pi) //Current and Speed PI Modes Enabled
					{
					iq_ref += ol_iqref_increase;
					speed_pid.PI_sat = iq_ref;
					speed_pid.I_prev = iq_ref;
					}
				else if (!enable_current_pi && enable_speed_pi) //Speed PI Mode Only
					{
					pwm_duty += ol_iqref_increase >> 16;
					speed_pwm_pid.PI_sat = pwm_duty;
					speed_pwm_pid.I_prev = pwm_duty;
					}
				else if (enable_current_pi && !enable_speed_pi) //Current PI Only
					{
					iq_ref += ol_iqref_increase;
					speed_pid.PI_sat = iq_ref;
					speed_pid.I_prev = iq_ref;
					}
				else 											//PWM Mode
					{
					pwm_duty += ol_iqref_increase >> 16;
					}
				}
		break;

		case SMS_Start_Motor_6S:

			speed_ref_hz = ol_start_hz;
			speed_ref_ticks = HertzToTicks((speed_ref_hz << 16), (TIMER_D_FREQ_F16 >> timer_d_div)) >> 16;

			TMRD_State = TimerD_SixStepOL;
			PAC55XX_TIMERD->PRD.w = speed_ref_ticks >> 16;
			PAC55XX_TIMERD->CTL.BASEIE = 1;										// Enable Timer D Base Interrupt
			PAC55XX_TIMERD->INT.BASEIF = 1;
			PAC55XX_TIMERD->CTL.MODE = TxCTL_MODE_UP;
			NVIC_EnableIRQ(TimerD_IRQn);

			SMS_State = SMS_Accel_Motor_6S;
			SMS_Counter = 0;
			avg_speed_index = 0;
		break;

		case SMS_Accel_Motor_6S:
			SMS_Counter++;
			if (SMS_Counter >= ol_accel_period)
				{
				SMS_Counter = 0;
				speed_ref_hz += ol_accel_increase;

				if (speed_ref_hz < ol_switchover_hz)
					{
					speed_ref_ticks = HertzToTicks(speed_ref_hz << 16, (TIMER_D_FREQ_F16 >> timer_d_div)) >> 16;
					motorspeed = speed_ref_ticks;
					pac5xxx_timer_base_config(TimerD, speed_ref_ticks, 0, TxCTL_MODE_UP, 0);
					}
				else if ((speed_ref_hz >= ol_switchover_hz) & switchover)
					{
					speed_ref_hz = ol_switchover_hz;
					TMRD_State = TimerD_Switchover;

					if (enable_current_pi && enable_speed_pi)
						{
						speed_pid.PI_sat = iq_ref;
						speed_pid.I_prev = iq_ref;
						}
					else if (!enable_current_pi && enable_speed_pi)
						{
						speed_pwm_pid.PI_sat = iq_pid.min_value;
						speed_pwm_pid.I_prev = iq_pid.min_value;
						}
					else if (enable_current_pi && !enable_speed_pi)
						{
						//iq_ref = start_iq_ref << 16;
						//iq_pid.PI_sat = iq_pid.min_value;
						//iq_pid.I_prev = iq_pid.min_value;
						}
					else //(!enable_current_pi && !enable_speed_pi)
						{
						//pwm duty comes from GUI
						}


					char i;
		        	avg_speed = 0;
		        	for (i=0;i<=5;i++)
		        		{
		        		avg_speed_array[i] = motorspeed;
		        		}
					avg_speed = motorspeed;

					SMS_State = SMS_Speed_Control_Loop;
					ADC_Counter = 0;
					ADCSM_State = ADCSM_IRegulate;
					}
				else
					{
					speed_ref_hz = ol_switchover_hz;
					speed_ref_ticks = HertzToTicks(speed_ref_hz << 16, (TIMER_D_FREQ_F16 >> timer_d_div)) >> 16;
					pac5xxx_timer_base_config(TimerD, speed_ref_ticks, 0, TxCTL_MODE_UP, 0);
					}
				}
		break;

		case SMS_Speed_Control_Loop:
			tmp_cl_accel++;
			if ((tmp_cl_accel >= cl_accel_period) && (speed_ref_hz != speed_ref_command_hz))
				{
				if (speed_ref_hz < speed_ref_command_hz)
					{
					speed_ref_hz += cl_accel_increase;
					if (speed_ref_hz > speed_ref_command_hz)
						{
						speed_ref_hz = speed_ref_command_hz;
						}
					}
				else if (speed_ref_hz > speed_ref_command_hz)
					{
					speed_ref_hz -= cl_accel_increase;
					if (speed_ref_hz < speed_ref_command_hz)
						{
						speed_ref_hz = speed_ref_command_hz;
						}
					}
				speed_ref_ticks = HertzToTicks((speed_ref_hz << 16), (TIMER_D_FREQ_F16 >> timer_d_div));
				tmp_cl_accel = 0;
				}
#if MOTORPWM_RCPWM
				pwm_duty = mtrpwm_width;
#else
			if (enable_current_pi && enable_speed_pi)
				{
				pid_run(&speed_pid, fix16_sub((avg_speed << 16), speed_ref_ticks));
				iq_ref = speed_pid.PI_sat;
				}
			else if (!enable_current_pi && enable_speed_pi)
				{
				pid_run(&speed_pwm_pid, fix16_sub((avg_speed << 16), speed_ref_ticks));
				pwm_duty = speed_pwm_pid.PI_sat >> 16;
				}
			else if (enable_current_pi && !enable_speed_pi)
				{
				//iq_ref comes from GUI
				}
			else //(!enable_current_pi && !enable_speed_pi)
				{
				//pwm duty comes from GUI
				}
#endif

			if (speed_ref_command_hz < RC_SPEED_TH_HZ_OFF)
				{
				SMS_State = SMS_Brake_Decel;
				SMS_Counter = 50;
				}
			break;


		case SMS_Brake_Decel:
			tmp_cl_accel++;
			if ((tmp_cl_accel >= cl_accel_period))
				{
				if (speed_ref_hz > BRAKE_SPEED_HZ)
					{
					speed_ref_hz -= cl_accel_increase;
					if (speed_ref_hz <= BRAKE_SPEED_HZ)
						{
						NVIC_DisableIRQ(TimerD_IRQn);
						NVIC_DisableIRQ(TimerC_IRQn);

						PAC55XX_TIMERD->CTL.BASEIE = 0;
						PAC55XX_TIMERD->CCTL0.CCINTEN = 0;

						//PAC5XXX_GPIOA->OUT.b = 0x07;//0;
						//PAC5XXX_GPIOA->PSEL.s = 0x0000;//15; 		//0x0595;

					#if defined (PAC5556)
						//PAC5XXX_GPIOA->OUT.b &= ~0x80;
						//PAC5XXX_GPIOD->PSEL.s &= ~0x4000;
					#endif

						pwm_duty = 50;
						SMS_Counter = 0;
						SMS_State = SMS_Brake_Apply;

						closed_loop_speed_hz = START_SPEED_HZ << 16;
						speed_ref_ticks = HertzToTicks(closed_loop_speed_hz, (TIMER_D_FREQ_F16 >> timer_d_div));
						speed_ref_command_hz = START_SPEED_HZ;
						speed_ref_hz = speed_ref_command_hz;
						}
					}
				speed_ref_ticks = HertzToTicks((speed_ref_hz << 16), (TIMER_D_FREQ_F16 >> timer_d_div));
				tmp_cl_accel = 0;
				}

			if (enable_current_pi && enable_speed_pi)
				{
				pid_run(&speed_pid, fix16_sub((avg_speed << 16), speed_ref_ticks));
				iq_ref = speed_pid.PI_sat;
				}
			else if (!enable_current_pi && enable_speed_pi)
				{
				pid_run(&speed_pwm_pid, fix16_sub((avg_speed << 16), speed_ref_ticks));
				pwm_duty = speed_pwm_pid.PI_sat >> 16;
				}
			else if (enable_current_pi && !enable_speed_pi)
				{
				//iq_ref comes from GUI
				}
			else //(!enable_current_pi && !enable_speed_pi)
				{
				//pwm duty comes from GUI
				}
			break;

		case SMS_Brake_Apply:
			SMS_Counter++;
			if (SMS_Counter >= 25)
				{
				if (app_status & status_motor_enabled)
					{
					motor_pwm_disable();
					}
				SMS_State = SMS_Wait_Radio_Command;
				}
			break;

		case SMS_Brake_End:
			SMS_Counter++;
			if (SMS_Counter >= 500)
				{

				}
			break;


		}

}
#endif
