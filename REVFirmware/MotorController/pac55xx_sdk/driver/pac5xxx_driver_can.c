/**************************************************************************//**
 * @file     pac5xxx_driver_can.c
 * @brief    Driver functions for the PAC55XX CAN Peripheral
 *
 * @note
 * Copyright (C) 2015-2019, Qorvo, Inc.
 *
 * THIS SOFTWARE IS SUBJECT TO A SOURCE CODE LICENSE AGREEMENT WHICH PROVIDES,
 * AMONG OTHER THINGS:  (i) THAT IT CAN BE USED ONLY TO ADAPT THE LICENSEE'S
 * APPLICATION TO PAC PROCESSORS SUPPLIED BY QORVO, INC.;
 * (ii) THAT  IT IS PROVIDED "AS IS" WITHOUT WARRANTY;  (iii) THAT
 * QORVO, INC. IS NOT LIABLE FOR ANY INDIRECT DAMAGES OR FOR DIRECT
 * DAMAGES EXCEEDING US$1,500;  AND (iv) THAT IT CAN BE DISCLOSED TO AND USED
 * ONLY BY CERTAIN AUTHORIZED PERSONS.
 *
 ******************************************************************************/
 
#include "pac5xxx_driver_can.h"


RAMFUNC_CAN void pac5xxx_can_io_config(void)  //TODO: make it configurable depending on CAN Pins selected
{
	// Select CAN peripheral on PE2 and PE3
	PAC55XX_GPIOE->MODE.P2 = 3;
	PAC55XX_GPIOE->MODE.P3 = 1;	
    PAC55XX_SCC->PEMUXSEL.w &= 0xFFFFF0FF;          // Clear Port E Pin 2 selection
    PAC55XX_SCC->PEMUXSEL.w |= 0x00000600;          // Set Port E Pin 2 CANRXD	
    PAC55XX_SCC->PEMUXSEL.w &= 0xFFFF0FFF;          // Clear Port F Pin 3 selection
    PAC55XX_SCC->PEMUXSEL.w |= 0x00006000;          // Set Port E Pin 3 CANTXD	
}
