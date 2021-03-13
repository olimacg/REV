/****************************************************************************
 * @file     isr_adc.c
 * @brief    ADC Interrupt Support
 * @date     13 February 2015
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

/**
 * @brief ADC Interrupt Handler routine
 */
APP_RAMFUNC void ADC_IRQHandler(void)
{
	//single_shunt_current = PAC55XX_ADC->DTSERES2.VAL + PAC55XX_ADC->DTSERES4.VAL + PAC55XX_ADC->DTSERES6.VAL;

	switch (ADCSM_State)
		{
		case ADCSM_Idle:
			//Do Nothing
		break;
		case ADCSM_Calibrate:
			phase_u_offset += U1_ADC_VAL;
			phase_v_offset += V1_ADC_VAL;
			phase_w_offset += W1_ADC_VAL;

			ADC_Counter--;
			if (!ADC_Counter)
				{
				ADCSM_State = ADCSM_Idle;
				phase_u_offset = phase_u_offset >> 9;
				phase_v_offset = phase_v_offset >> 9;
				phase_w_offset = phase_w_offset >> 9;
				}
		break;

		case ADCSM_SineWave:
			//During Sine Wave Commutation, TIMERD Controls PWM Duty
		break;

		case ADCSM_SixStep:
			if (pwm_duty == 0) pwm_duty++;

			PAC55XX_TIMER_SEL->CCTR4.CTR = pwm_duty;				//Configure U Phase Duty Cycle
			PAC55XX_TIMER_SEL->CCTR5.CTR = pwm_duty;				//Configure V Phase Duty Cycle
			PAC55XX_TIMER_SEL->CCTR6.CTR = pwm_duty;				//Configure W Phase Duty Cycle
			break;

		case ADCSM_IRegulate:
			__disable_irq();

			if (PAC55XX_GPIOB->OUT.w & 0x01)
				{
				single_shunt_current = U1_ADC_VAL - phase_u_offset;
				}
			else if (PAC55XX_GPIOB->OUT.w & 0x02)
				{
				single_shunt_current = V1_ADC_VAL - phase_v_offset;
				}
			else if (PAC55XX_GPIOB->OUT.w & 0x04)
				{
				single_shunt_current = W1_ADC_VAL - phase_w_offset;
				}

			__enable_irq();

			ADC_Counter++;
			if (ADC_Counter >= 1)
				{
				ADC_Counter = 0;
				if (tmp_enable_current_pi)
					{
					tmp_pi_debug_index = 0;
					pid_run(&iq_pid, fix16_sub(iq_ref, (single_shunt_current << 16)));
					pwm_duty = iq_pid.PI_sat >> 16;
					}
				}
			if (pwm_duty == 0) pwm_duty++;
			
			PAC55XX_TIMER_SEL->CCTR4.CTR = pwm_duty;				//Configure U Phase Duty Cycle
			PAC55XX_TIMER_SEL->CCTR5.CTR = pwm_duty;				//Configure V Phase Duty Cycle
			PAC55XX_TIMER_SEL->CCTR6.CTR = pwm_duty;				//Configure W Phase Duty Cycle

		break;
		}
	PAC55XX_ADC->ADCINT.ADCIRQ0IF = 1;
}
