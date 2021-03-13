/****************************************************************************
 * @file     hs_state_machine.c
 * @brief    State Machine For The Hall Sensor Based BLDC Application
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

#ifdef HALL_SENSOR_APP_UNDEF
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

#if POT_CONTROL
	case SMS_Idle:
		if (!(app_status & status_motor_enabled) && (pot_volts_command > TURN_ON_THRESHOLD_MAX))
			{
			SMS_State = SMS_Align_6S;
			}
		break;

#else
				case SMS_Idle:
					//do nothing
				break;
#endif

	case SMS_Align_6S:

		app_status &= ~status_motor_stalled;
		app_status |= status_motor_enabled;

		PAC55XX_TIMERC->CTL.MODE = TxCTL_MODE_DISABLED;
		PAC55XX_TIMERC->CTL.CLR = 1;
		PAC55XX_TIMERD->CTL.MODE = TxCTL_MODE_DISABLED;
		PAC55XX_TIMERD->CTL.CLR = 1;

#if ANGLE_ADVANCE
		PAC55XX_TIMERD->CTL.BASEIE = 0;
#endif

		if (enable_current_pi && enable_speed_pi)
			{
			iq_ref = start_iq_ref << 16;
			iq_pid.PI_sat = iq_pid.min_value;
			iq_pid.I_prev = iq_pid.min_value;

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
			iq_ref = start_iq_ref << 16;
			iq_pid.PI_sat = iq_pid.min_value;
			iq_pid.I_prev = iq_pid.min_value;
			}
		else //(!enable_current_pi && !enable_speed_pi)
			{
			//pwm duty comes from GUI
			}

		speed_ref_hz = ol_start_hz;

	  	// Initialize Average Speed Array
		// This is done after first commutation to ensure array is filled with good values.
		// The motor-speed value on the first commutate() execution is not meaningful as it has no point of reference
		motorspeed = HertzToTicks((speed_ref_hz << 16), (TIMER_D_FREQ_F16 >> timer_d_div)) >> 16;
		uint32_t i;
		for (i=0;i<=5;i++)
	      	{
	   		avg_speed_array[i] = motorspeed;
	      	}

		avg_speed_index = 0;

		PAC55XX_TIMERC->CTL.CLR = 0;
		PAC55XX_TIMERC->INT.BASEIF = 1;
		PAC55XX_TIMERC->CTL.BASEIE = 1;
		PAC55XX_TIMERD->CTL.CLR = 0;
		PAC55XX_TIMERD->INT.BASEIF = 1;
		PAC55XX_TIMERD->CTL.BASEIE = 1;
		PAC55XX_TIMERC->CTL.MODE = TxCTL_MODE_UP;
		PAC55XX_TIMERD->CTL.MODE = TxCTL_MODE_UP;

		#if ANGLE_ADVANCE
			aa_commutate(firstcomm);
		#else
			commutate(firstcomm);
		#endif

		app_status |= status_closed_loop;

#if POT_CONTROL
		SMS_State = SMS_Pot_Control_Loop;
#else
		SMS_State = SMS_Speed_Control_Loop;
#endif

		tmp_enable_current_pi = enable_current_pi;
		ADC_Counter = 0;
		ADCSM_State = ADCSM_IRegulate;
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

	case SMS_Brake_Decel:
		tmp_cl_accel++;
		if ((tmp_cl_accel >= cl_accel_period))
			{
			if (speed_ref_hz > speed_ref_command_hz)
				{
				speed_ref_hz -= cl_accel_increase;
				if (speed_ref_hz <= speed_ref_command_hz)
					{
					SMS_Counter = 0;
					SMS_State = SMS_Brake_Apply;
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
		if (SMS_Counter >= 250)
			{
			app_status &= ~(status_motor_enabled + status_closed_loop);
			//PAC5XXX_GPIOA->PSEL.s = 0x0595;	        // All drives are PWM
			SMS_Counter = 0;
			SMS_State = SMS_Brake_End;
			}
		break;

	case SMS_Brake_End:
		SMS_Counter++;
		if (SMS_Counter >= 500)
			{
			motor_pwm_disable();
			SMS_State = SMS_Idle;
			}
		break;

	case SMS_Auto_On_Wait:
		SMS_Counter--;
		if (SMS_Counter == 0)
			{
			SMS_State = SMS_Align;
			}
		break;

	case SMS_Pot_On_Wait:
		SMS_Counter--;
		if (SMS_Counter == 0)
			{
			SMS_State = SMS_Idle;
			}
		break;

	case SMS_Pot_Control_Loop:
		tmp_cl_accel++;

		if (pot_volts_command > TURN_ON_THRESHOLD_MIN)
			{
			if ((tmp_cl_accel >= cl_accel_period) && (pwm_duty != pot_volts_command))
				{
				if (pwm_duty < pot_volts_command)
					{
					pwm_duty += cl_accel_increase;
					if (pwm_duty > pot_volts_command)
						{
						pwm_duty = pot_volts_command;
						}
					}
				else if (pwm_duty > pot_volts_command)
					{
					pwm_duty -= cl_accel_increase;
					if (pwm_duty < pot_volts_command)
						{
						pwm_duty = pot_volts_command;
						}
					}
				tmp_cl_accel = 0;
				}
			}
		else
			{
			motor_pwm_disable();
			}

		break;
	}
}
#endif
