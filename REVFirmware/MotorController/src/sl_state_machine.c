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

#ifdef SENSORLESS_APP

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
				//do nothing
				break;

				case SMS_Align:
								
				app_status &= ~status_motor_stalled;
				
#ifdef PAC5556
				PAC55XX_SCC->PBMUXSEL.w =  MOTOR_OUT_PORTB_ALL_PWMS;				
				PAC55XX_SCC->PCMUXSEL.w =  MOTOR_OUT_PORTC_ALL_PWMS;								
#else
				PAC55XX_SCC->PBMUXSEL.w =  MOTOR_OUT_PORTB_ALL_PWMS;
#endif
				

				
	#if BLDC_DIRECTION
				sine_index = 0;
	#else
				sine_index = 359;
	#endif
				
				ol_iqref_increase = fix16_mul_new_16_16(pwm_period_ticks << 16, sine_scale_max);
				ol_iqref_increase = fix16_div(ol_iqref_increase, align_time_ms << 16) >> 16;
				sine_scale = 0;

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

				blanking_cycles = START_BLANKING_CYCLES;
				good_samples = START_GOOD_SAMPLES;

				SMS_Counter = 0;
				SMS_State = SMS_Align_Wait;
				app_status |= status_motor_enabled;
				open_loop = 1;
				ADCSM_State = ADCSM_SineWave;
				
				PAC55XX_TIMERD->CTL.MODE = 0;
				
				break;

				case SMS_Align_Wait:
					
				SMS_Counter ++;
				if (SMS_Counter > align_time_ms)
					{
					SMS_Counter = 0;
					SMS_State = SMS_Start_Motor;
					}
				else
					{
					sine_scale += ol_iqref_increase;

					fix16_t temp0, temp1, temp2;

					temp0 = fix16_mul_new_16_16(sine_wave_3phase[sine_index][0], sine_scale);
					temp1 = fix16_mul_new_16_16(sine_wave_3phase[sine_index][1], sine_scale);
					temp2 = fix16_mul_new_16_16(sine_wave_3phase[sine_index][2], sine_scale);
						
					temp0++;
					temp1++;
					temp2++;

					PAC55XX_TIMER_SEL->CCTR4.CTR = temp0;
					PAC55XX_TIMER_SEL->CCTR5.CTR = temp1;
					PAC55XX_TIMER_SEL->CCTR6.CTR = temp2;
					}
				break;

				case SMS_Align_Hold:
				SMS_Counter++;
				if (SMS_Counter > 10)
					{
					SMS_State = SMS_Start_Motor;
					}
				break;

				case SMS_Start_Motor:

				speed_ref_hz = ol_start_hz;
				speed_ref_ticks = HertzToTicksSine((speed_ref_hz << 16), (TIMER_D_FREQ_F16 >> timer_d_div));

				PAC55XX_TIMERD->PRD.w = speed_ref_ticks >> 16;
				PAC55XX_TIMERD->CTL.BASEIE = 1;										// Enable Timer D Base Interrupt
				PAC55XX_TIMERD->INT.BASEIF = 1;
				PAC55XX_TIMERD->CTL.MODE = TxCTL_MODE_UP;

				NVIC_EnableIRQ(TimerD_IRQn);
				SMS_State = SMS_Accel_Motor;
				SMS_Counter = 0;
				avg_speed_index = 0;
				break;

				case SMS_Accel_Motor:
				SMS_Counter++;
				if (SMS_Counter >= ol_accel_period)
					{
					SMS_Counter = 0;
					speed_ref_hz += ol_accel_increase;
						
					if (speed_ref_hz < ol_switchover_hz)
						{
						speed_ref_ticks = HertzToTicksSine(speed_ref_hz << 16, (TIMER_D_FREQ_F16 >> timer_d_div));
						pac5xxx_timer_base_config(TimerD, (speed_ref_ticks >> 16), 0, TxCTL_MODE_UP, 0);
						motorspeed = HertzToTicks(speed_ref_hz << 16, (TIMER_D_FREQ_F16 >> timer_d_div)) >> 16;	
						}					
					else if ((speed_ref_hz >= ol_switchover_hz) & switchover)
						{
						speed_ref_hz = ol_switchover_hz;
						PAC55XX_TIMERD->CTL.BASEIE = 0;	
						ADC_Counter = 0;
						ADCSM_State = ADCSM_IRegulate;
						tmp_enable_current_pi = enable_current_pi;
							
						PAC55XX_GPIOB->OUT.w = c_pwm_io_state[motordir][sl_current_state];
#ifdef PAC5556
						PAC55XX_SCC->PBMUXSEL.w = psel_mask_pbmux[motordir][sl_current_state];	        // Set peripheral select state
						PAC55XX_SCC->PCMUXSEL.w = psel_mask_pcmux[motordir][sl_current_state];	        // Set peripheral select state
#else
						PAC55XX_SCC->PBMUXSEL.w = psel_mask[motordir][sl_current_state];	        // Set peripheral select state
#endif
						sl_current_state++;

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
						
						PAC55XX_TIMERD->CCTR0.CTR = motorspeed >> 1;
						PAC55XX_TIMERD->PRD.w = 0xFFFF;
						PAC55XX_TIMERD->CTR.w = 0;

			        	tmp_blanking_cycles = blanking_cycles;
						PAC55XX_TIMERC->CCTL0.CCINTEN = 0;					//Disable TIMER C CCR0 ISR
						PAC55XX_TIMERC->INT.CCR0IF = 1;						//Clear TIMER C CCR0 Flag
						PAC55XX_TIMERD->INT.CCR0IF = 1;						//Clear TIMER D CCR0 Flag
						PAC55XX_TIMERD->CCTL0.CCINTEN = 1;					//Enable TIMER D CCR0 Interrupt (to cause commutation at required time)

						NVIC_EnableIRQ(TimerC_IRQn);

						char i;
			        	avg_speed = 0;
			        	for (i=0;i<=5;i++)
			        		{
			        		avg_speed_array[i] = motorspeed;
			        		}
						avg_speed = motorspeed;
						open_loop = 0;
							
						app_status |= status_closed_loop;
						SMS_State = SMS_StartUp;
						}
					else
						{
						speed_ref_hz = ol_switchover_hz;
						speed_ref_ticks = HertzToTicksSine(speed_ref_hz << 16, (TIMER_D_FREQ_F16 >> timer_d_div));
						pac5xxx_timer_base_config(TimerD, (speed_ref_ticks >> 16), 0, TxCTL_MODE_UP, 0);
						}
					}
					
				break;

				case SMS_StartUp:
				SMS_Counter++;
				if (SMS_Counter > OL_CL_TRANSITION_MS)
					{
					SMS_State = SMS_Speed_Control_Loop;
					blanking_cycles = RUN_BLANKING_CYCLES;
					good_samples = RUN_GOOD_SAMPLES;
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

				case SMS_Align_Hold_6S:			//Place holder if more Align Time needed
				SMS_Counter++;
				if (SMS_Counter > 10)
					{
					SMS_State = SMS_Start_Motor_6S;
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

#if POT_CONTROL
							SMS_State = SMS_Pot_Control_Loop;
#else
							SMS_State = SMS_Speed_Control_Loop;

#endif
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
					if (SMS_Counter >= 150)
						{
						motor_pwm_disable();
						SMS_State = SMS_Idle;
						}
					break;

				case SMS_Brake_End:
					SMS_Counter++;
					if (SMS_Counter >= 500)
						{

						}
					break;

				case SMS_Beep_ON:
					app_status &= ~status_motor_stalled;

					PAC55XX_TIMER_SEL->CCTR4.CTR = beep_pwm_dc;
					PAC55XX_TIMER_SEL->CCTR5.CTR = beep_pwm_dc;
					PAC55XX_TIMER_SEL->CCTR6.CTR = beep_pwm_dc;

					sl_current_state = 0;

					PAC55XX_GPIOB->OUT.w = c_pwm_io_state[motordir][sl_current_state];
#ifdef PAC5556
					PAC55XX_SCC->PBMUXSEL.w = psel_mask_pbmux[motordir][sl_current_state];	        // Set peripheral select state
					PAC55XX_SCC->PCMUXSEL.w = psel_mask_pcmux[motordir][sl_current_state];	        // Set peripheral select state
#else
					PAC55XX_SCC->PBMUXSEL.w = psel_mask[motordir][sl_current_state];	        // Set peripheral select state
#endif	
					sl_current_state++;

					app_status |= status_motor_enabled;
					open_loop = 1;
					ADC_Counter = 0;
					ADCSM_State = ADCSM_IRegulate;

					speed_ref_hz = beep_freq_hz;
					speed_ref_ticks = HertzToTicks((speed_ref_hz << 16), (TIMER_D_FREQ_F16 >> timer_d_div));
					pac5xxx_timer_base_config(TimerD, (speed_ref_ticks >> 16), 0, TxCTL_MODE_UP, 0);
					PAC55XX_TIMERD->CTL.BASEIE = 1;										// Enable Timer D Base Interrupt
					NVIC_EnableIRQ(TimerD_IRQn);
					SMS_State = SMS_Idle;
					break;
				case SMS_Beep_OFF:
					beep_off();
					break;
				case SMS_Diag_Init:
					diag_note_offset = 0;
					SMS_Counter = 50;
					SMS_State = SMS_Diag_On_Wait;
					beep_on(diag_tunes[diag_message_offset][diag_note_offset]);
					break;
				case SMS_Diag_On_Wait:
					SMS_Counter--;
					if (SMS_Counter == 0)
						{
						beep_off();
						SMS_Counter = 10;
						SMS_State = SMS_Diag_Off_Wait;
						}
					break;
				case SMS_Diag_Off_Wait:
					SMS_Counter--;
					if (SMS_Counter == 0)
						{
						diag_note_offset++;
						if (diag_note_offset <= 3)
							{
							SMS_Counter = 50;
							SMS_State = SMS_Diag_On_Wait;
							beep_on(diag_tunes[diag_message_offset][diag_note_offset]);
							}
						else
							{
							SMS_State = SMS_Idle;
							}
						}
					break;
				case SMS_Auto_On_Wait:
					SMS_Counter--;
					if (SMS_Counter == 0)
						{
						SMS_State = SMS_Align_6S;
						}
					break;
			}
}

#endif
