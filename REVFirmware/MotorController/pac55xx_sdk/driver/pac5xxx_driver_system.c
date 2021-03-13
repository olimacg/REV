/**************************************************************************//**
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

#include "PAC5XXX_driver_system.h"


RAMFUNC_SYSTEM void pac5xxx_sys_pll_config_enable(uint32_t pllindiv, uint32_t pllfbdiv, uint32_t plloutdiv)
{
    //  Configure PLL control reg and enable PLL
    PAC55XX_SCC->CCSPLLCTL.PLLEN = 0;
    PAC55XX_SCC->CCSPLLCTL.PLLFBDIV = pllfbdiv;
    PAC55XX_SCC->CCSPLLCTL.PLLOUTDIV = plloutdiv;
    PAC55XX_SCC->CCSPLLCTL.PLLINDIV = pllindiv;
    PAC55XX_SCC->CCSPLLCTL.PLLEN = 1;
    // Wait for Lock
    while(PAC55XX_SCC->CCSPLLCTL.PLLLOCK == 0);    
}


//=====================================================================================
// pac_recovery() provides a mechanism to recover a misconfigured PAC device
// Ground PE3 at power up to add a 3 second delay so that the Flash can be erased
//=====================================================================================
// NOTE:  #define PAC_RECOVERY should be commented out for a production system
// When PAC_RECOVERY is defined, then a 3 Second delay will be added if PE3 is connected to ground at PAC power up
// The 3 second delay will give the user time to erase the device using SWD, which will eliminate the code that is misconfiguring the device
// On PAC55xx EVKs, PE3 is normally the GUI UART PAC RX input pin (Host TX); a delay will not normally be inserted even with GUI running
#define PAC_RECOVERY    // Define to enable PAC recovery mechanism

#ifdef PAC_RECOVERY

#warning PAC_RECOVERY enabled; Ground PE3 to add 3 second boot delay to allow recovery of misconfigured device; remove for production system

void pac_recovery(void)
{
    volatile uint32_t i;
    uint32_t perform_delay = 1;

    // PE3 is already an input, just need to Enable Pull Up
    PAC55XX_SCC->PEPUEN.P3 = 1;                         // Enable Pull Up

    // If PE3 is high a delay is not needed
    // Must check over the length of a full UART packet at 115kbps in case UART is talking and making PE3 low for short durations
    for (i=0; i < 40; i++)  // Value of 40 = at least 145 uS, which is enough for 1 UART packet (78uS)
    {
        // If PE3 is high at any time, then a delay is not needed
        if (PAC55XX_GPIOE->IN.P3 == 1)
        {
            perform_delay = 0;
        }
    }

    // If PE3 was low, delay required, add 3 second delay
    if(perform_delay)
    {
        pac_delay_asm(1400000);
    }

    // Restore PE3 to reset state
    PAC55XX_SCC->PEPUEN.P3 = 0;                         // Disable Pull Up
}

#else

void pac_recovery(void)
{
    // No Recovery required for production system
}

#endif  // #ifdef PAC_RECOVERY
