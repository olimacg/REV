/****************************************************************************
 * @file     rc_def.h
 * @brief    Public interface to sensorless BLDC motor with RC Control support
 * @date     3 September 2015
 *
 * @note
 * Copyright (C) 2015-2019, Qorvo Inc.
 *
 * THIS SOFTWARE IS SUBJECT TO A SOURCE CODE LICENSE AGREEMENT WHICH PROVIDES,
 * AMONG OTHER THINGS:  (i) THAT IT CAN BE USED ONLY TO ADAPT THE LICENSEE'S
 * APPLICATION TO PAC PROCESSORS SUPPLIED BY QORVO;
 * (ii) THAT  IT IS PROVIDED "AS IS" WITHOUT WARRANTY;  (iii) THAT
 * QORVO, INC. IS NOT LIABLE FOR ANY INDIRECT DAMAGES OR FOR DIRECT
 * DAMAGES EXCEEDING US$1,500;  AND (iv) THAT IT CAN BE DISCLOSED TO AND USED
 * ONLY BY CERTAIN AUTHORIZED PERSONS.
 ******************************************************************************/
#ifndef RC_DEF_H
#define RC_DEF_H

#define	MOTORPWM_RCPWM		0

typedef enum {
	RCGone = 0,
	RCPresent
} RCState;

//***********************************************************************
//RC Code Block Definition
#define	RCPWM_WIDTH_MIN		1563				//Min Number of Timer B ticks. 1563 * 640 ns = 1.0 ms
#define	RCPWM_WIDTH_MID		2343				//Min Number of Timer B ticks. 2343 * 640 ns = 1.5 ms
#define	RCPWM_WIDTH_MAX		3125				//Max Number of Timer B ticks. 3125 * 640 ns = 2.0 ms

#define	RC_PWM_50HZ			0					//1 for 50 HZ (20 ms period)
#define	RC_PWM_400HZ		1					//1 for 400 Hz (2.5 ms Period)
#define	RC_PWM_40HZ			0					//1 for 100 Hz (10 ms Period)

#if RC_PWM_50HZ 								//RC PWM Frequency is 50 Hz (every 20 to 21 ms)
	#define	RCPWM_PERIOD_HI		33000			//21.12 ms (33000 * 640 ns = 21.12 ms)
	#define	RCPWM_PERIOD_LO		31000			//19.84 ms (31000 * 640 ns = 19.84 ms)
#elif RC_PWM_400HZ								//RC PWM Frequency is 400 Hz (every 2.5 ms)
	#define	RCPWM_PERIOD_HI		4000			//2.56 ms (4000 * 640 ns = 2.56 ms)
	#define	RCPWM_PERIOD_LO		3800			//2.43 ms (3800 * 640 ns = 2.432 ms)
#else											//RC PWM Frequency is 100 Hz (every 10 ms)
	#define	RCPWM_PERIOD_HI		17187			//11 ms (17187 * 640 ns = 11 ms)
	#define	RCPWM_PERIOD_LO		14062			//9 ms (14062 * 640 ns = 2.432 ms)
#endif

#define MAX_PWM_PERIOD_COUNTS		5
#define	RC_MAX_SPEED_HZ				2000
#define	RCPWM_NORMALIZE_CONSTANT	FIX16(RC_MAX_SPEED_HZ/1563)	//2500 / 1563 = 1.599 in FIX16
#define	MTRPWM_MAX_DC				1000
#define	RCPWM_NORMALIZE_DC_CONSTANT	FIX16(MTRPWM_MAX_DC/1563)	//2450 / 1563 = 1.5674 in FIX16
#define FREQ_INV					FIX16(1/1562)				//1/1562 = 0.00064 in FIX16
#define	RC_SPEED_TH_HZ_ON			200
#define	RC_SPEED_TH_HZ_OFF			150
#define	FULL_THROTTLE_RC			1							//1 for Full Throttle style; 0 for Dual Direction/Speed Control

#if defined (PAC5523) || defined (PAC5524)
#define	RC_PWM_IN_GPIO			PAC55XX_GPIOC->IN.w
#elif defined (PAC5527)
#define	RC_PWM_IN_GPIO			PAC55XX_GPIOD->IN.w
#endif

EXTERN uint8_t good_pulse;
EXTERN uint8_t rc_status;

EXTERN uint16_t prev_rcpwm_time_rise;
EXTERN uint16_t rcpwm_time_rise, rcpwm_time_fall, rcpwm_width;
EXTERN uint16_t rcpwm_period;
EXTERN uint16_t mtrpwm_width;
EXTERN int16_t rcpwm_period_counter;
EXTERN uint32_t command_mdir;
EXTERN uint32_t legal_fedge;

EXTERN fix16_t rc_command;
EXTERN fix16_t rc_width_ms;

EXTERN uint32_t legal_fedge;
#endif
