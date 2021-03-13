



#define INCLUDE_EXTERNS
#include "bldc_common.h"

void configure_timer_b_compare_mode(void)
{
    uint32_t pclk = 300000000/2;        // PCLK assumed to be 150 MHz
    uint16_t period;

    // Configure Timer C Controls
    period = pclk / 10000;                                                      // Timer Period will result in 10 kHz
    pac5xxx_timer_clock_config(TimerB, TXCTL_CS_ACLK, TXCTL_PS_DIV1);           // Configure timer clock input for ACLK, /1 divider
    pac5xxx_timer_base_config(TimerB, period, 0, TxCTL_MODE_DISABLED, 0);     // Configure timer frequency and count mode

    PAC55XX_TIMERB->CTL.PRDLATCH = TXCTL_PRDLATCH_TXCTR_IM;                     // 00b--> copied TAPRD into the shadow registers when TACTR from 1 to 0(or from TAPRD to 0)  
                                                                                // 01b--> copied TAPRD into the shadow registers when TACTR from TAPRD-1 to TAPRD
                                                                                // 10b--> copied TAPRD into the shadow registers as soon as the TAPRD register is written

    PAC55XX_TIMERB->CCTR4.CTR = period >> 3;

    // Enable TACCR0 interrupt at highest priority
    PAC55XX_TIMERB->CCTL4.CCMODE = TXCCTL_CCMODE_COMPARE;                       // Set compare mode

    PAC55XX_TIMERB->CCTL4.CCINTEDGE = TXCCTL_CCINT_FALLING_EDGE;                // 0 -->rising edge interrupt
                                                                                // 1 -->falling edge interrupt
                                                                                // 2 -->rising and falling edge interrupt

    PAC55XX_TIMERB->CCTL4.CCLATCH = TXCCTL_CCLATCH_COMPARE_TXCTR_IM;            // 00b--> copied CTR4 into the shadow registers when TACTR from 1 to 0(or from TAPRD to 0) 
                                                                                // 01b--> copied CTR4 into the shadow registers when TACTR from TAPRD-1 to TAPRD 
                                                                                // 10b--> copied CTR4 into the shadow registers as soon as the TAPRD register is written

    PAC55XX_TIMERB->CTL.MODE = TxCTL_MODE_UPDOWN;

    PAC55XX_TIMERB->CCTL4.CCINTEN = 1;                                          // Enable interrupts on TCCCR0
    PAC55XX_TIMERB->INT.CCR4IF = 1;                                             // Clear PWMC0 interrupt flag 
    PAC55XX_TIMERB->CTL.BASEIE = 1;                                             // Enable base timer
    PAC55XX_TIMERB->INT.BASEIF = 1;                                             // Clear timer base interrupt flag
    NVIC_EnableIRQ(TimerB_IRQn);                                                // Enable TimerC interrupts
    NVIC_SetPriority(TimerB_IRQn ,1);                                           // Set TimerC Priority to 1

    PAC55XX_SCC->PCMUXSEL.P4 = 0x1;                                             // PC4 -->PWMB4
    PAC55XX_GPIOC->OUTMASK.P4 = 0;                                              // PC4 -->Output
}