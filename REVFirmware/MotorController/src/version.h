/****************************************************************************
 * @file     version.h
 * @brief    Version number for the SIMPLE BLDC Firmware
 * @date     12 May 2015
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

#ifndef VERSION_H
#define VERSION_H

#define VERSION_TYPE					2
#define VERSION_MAJOR					2
#define VERSION_MINOR					0
#define	PROJECT_VERSION_RELEASE_STATE	PAC55XX_RELEASED
#define PROJECT_VERSION_STR				"2019-09-10"

#endif

/*
 * Version History
 *
 *
	Version 2.2.0 (Release)
	1. Updated SDK to 1.5.1
	2. Added Stall Detection for the case in which a false BEMF detection takes place and the motor commutates at ultra high speeds with lowest PWM duty cycle

	Version 2.2.0A (In Development)
	1. Added Support for PAC5527
	2. Added Support for RC PWM Control

	Version 2.1.0
	1. Added support for PAC5224
	2. Moved OCP definitions to the board definitions in order to ensure the OCP levels move with the HW.
	3. Improved the ADC Sequencer table to ensure EMUX channels are sampled on the correct Sequencer Table Entry.
	4. Updated SDK Version to 1.4.0 (For added PAC5524 support)

	Version 2.0.1 - 2019-07-31 - IN DEVELOPMENT
	1. Fixed a problem with the Hall Sensor Startup with Angle Advance. Added an instruction on aa_commutate() to force a commutation scheduling which gets the motor started.
 
 	Version 2.0.0 - 2019-05-15 - IN DEVELOPMENT
 	1. Fixed a problem with angle Advance. TIMERD is used to cause commutation, as opposed to using the same TIMERC for commutation.

	Version 2.0.0
	1. Added support for PAC5556
	2. Added support for Hall Sensor Algorithm
	3. Updated SDK to version 1.2.0

	Version 1.2.0
	1. Unified PAC5523, PAC5532 and PAC5556 Code in a single project
	2. Updated SDK to version 1.1.0

	Version 1.1.0	10/31/2018
	1. (init.c) Fixed a bug with the Differential Amplifier Configuration having the LP and HP protection definitions inverted.
	2. (peripheral_init.c) made PORTC4/5/6 inputs to handle Hall Sensor data.
  
	Version 1.0.0 10/31/2017
	
	1. Initial Release
 *
 */
