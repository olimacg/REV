/****************************************************************************
 * @file     isr_TimerC.c
 * @brief    Timer C Interrupt Service Routine
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




APP_RAMFUNC void TimerC_IRQHandler(void)
{
  
  
	if (PAC55XX_TIMERC->INT.BASEIF)
		{
#if ANGLE_ADVANCE
		aa_commutate(pwmc_base);
#else
		commutate(pwmc_base);
#endif
		PAC55XX_TIMERC->INT.BASEIF = 1;
                
                
		}
	 if (PAC55XX_TIMERC->INT.CCR0IF)
		{
#if ANGLE_ADVANCE
		aa_commutate(pwmc0);
#else
		commutate(pwmc0);
#endif
		PAC55XX_TIMERC->INT.CCR0IF = 1;
		}
	 if (PAC55XX_TIMERC->INT.CCR1IF)
		{
#if ANGLE_ADVANCE
		aa_commutate(pwmc1);
#else
		commutate(pwmc1);
#endif
		PAC55XX_TIMERC->INT.CCR1IF = 1;
		}
	if (PAC55XX_TIMERC->INT.CCR2IF)
		{
#if ANGLE_ADVANCE
		aa_commutate(pwmc2);
#else
		commutate(pwmc2);
#endif
		PAC55XX_TIMERC->INT.CCR2IF = 1;
		}
	if (PAC55XX_TIMERC->INT.CCR3IF)
		{
		PAC55XX_TIMERC->INT.CCR3IF = 1;
		}
	if (PAC55XX_TIMERC->INT.CCR4IF)
		{
#if ANGLE_ADVANCE
		aa_commutate(pwmc4);
#else
		commutate(pwmc4);
#endif
		PAC55XX_TIMERC->INT.CCR4IF = 1;
		}
	if (PAC55XX_TIMERC->INT.CCR5IF)
		{
#if ANGLE_ADVANCE
		aa_commutate(pwmc5);
#else
		commutate(pwmc5);
#endif
		PAC55XX_TIMERC->INT.CCR5IF = 1;
		}
	if (PAC55XX_TIMERC->INT.CCR6IF)
		{
#if ANGLE_ADVANCE
		aa_commutate(pwmc6);
#else
		commutate(pwmc6);
#endif
		PAC55XX_TIMERC->INT.CCR6IF = 1;
		}
	if (PAC55XX_TIMERC->INT.CCR7IF)
		{
		PAC55XX_TIMERC->INT.CCR7IF = 1;
		}
}




