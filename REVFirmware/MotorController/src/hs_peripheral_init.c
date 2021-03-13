/****************************************************************************
 * 
 *                      Peripheral Initialization Function
 *
 * 
 ******************************************************************************/
#define INCLUDE_EXTERNS
#include "bldc_common.h"

#ifdef HALL_SENSOR_APP

/**
 * @brief  This function configures the PAC peripherals for this application
 *
 * @return none
 *
 */
void peripheral_init(void)
{
	// Set Flash Lock to allow write access to MEMCTL register for configuring clocks
    PAC55XX_MEMCTL->FLASHLOCK = FLASH_LOCK_ALLOW_WRITE_MEMCTL;      

    // Turn on Flache Cache
    PAC55XX_MEMCTL->MEMCTL.w &= ~(1 << 21);

    // Select 4 MHz CLKREF for Free Running Clock FRCLK
    PAC55XX_SCC->CCSCTL.FRCLKMUXSEL = CCSCTL_CLKIN_CLKREF;
 	pac5xxx_sys_pll_config_enable(4, 300, 0);                   // PLLCLK = 300 MHz = (4/4 * 300) /1
	
    // Configure SCLK=PLLCLK=300 MHz, HCLK=150 MHz, PCLK=50 MHz, ACLK=100 MHz and WaitStates;
    PAC55XX_SCC->CCSCTL.HCLKDIV = CCSCTL_HCLKDIV_DIV2;          // HCLK = 150 MHz = SCLK/2; when SCLK = PLLCLK
    PAC55XX_SCC->CCSCTL.ACLKDIV = CCSCTL_ACLKDIV_DIV3;          // ACLK = 100 MHz = SCLK/3; when SCLK = PLLCLK
	PAC55XX_SCC->CCSCTL.PCLKDIV = CCSCTL_PCLKDIV_DIV3;			// PCLK = 50 MHz = HCLK/3; when SCLK = PLLCLK
  	PAC55XX_MEMCTL->MEMCTL.WSTATE = 6 + 1;                      // Flash = 150/(5 + 1 Extra WS + 1) = 25 MHz + extra WS,   TODO: The 1 Extra WS might be able to be removed after testing
    PAC55XX_SCC->CCSCTL.SCLKMUXSEL = CCSCTL_SCLK_PLLCLK;        // SCLK = PLLCLK
    
    // Set MCLK for Flash write & erase in addition to read
    PAC55XX_MEMCTL->MEMCTL.MCLKDIV = MEMCTL_MCLK_DIV5;          // MCLK will = HCLK/5 when MCLKSEL = MEMCTL_MCLK_HCLKDIV
    PAC55XX_MEMCTL->MEMCTL.MCLKSEL = MEMCTL_MCLK_HCLKDIV;       // MCLK = HCLK/5 = 30 MHz; allows reading and writing of Flash  
    
    PAC55XX_MEMCTL->FLASHLOCK = 0;                              // Disallow write access to MEMCTL
    PAC55XX_MEMCTL->MEMCTL.CACHEDIS = 0;                        // MEMCTL.CACHEDIS, enable Flash CACHE


	// Configure Timer A parameters. Timer A runs at PLL Frequency (selected above).
	pac5xxx_timer_clock_config(TimerA, TXCTL_CS_ACLK, TXCTL_PS_DIV1);      		// Configure timer clock input for ACLK, /1 divider
	app_pwm_period = PWM_SWITCH_FREQ;											// Number of KHz
	pwm_period_ticks = TIMER_X_FREQ_CNV / (app_pwm_period);						// 50,000 / # KHz
	pac5xxx_timer_base_config(TimerA, pwm_period_ticks, 0, TxCTL_MODE_UP, 0);	// Configure Timer


	//Configure PWM Outputs
	PAC55XX_TIMER_SEL->CCTR4.CTR = pwm_period_ticks >> 1;
	PAC55XX_TIMER_SEL->CCTR5.CTR = pwm_period_ticks >> 1;
	PAC55XX_TIMER_SEL->CCTR6.CTR = pwm_period_ticks >> 1;

	// Configure Hardware Dead Time Generator
	dt_leading_ticks = DT_LED_TICKS;
	dt_trailing_ticks = DT_TED_TICKS;
	Set_Dead_Time();

	//To Drive the Motor PWM's and the Outputs
	// Write to the MODE only needs to be done once. They are always PUSH/PULL
	// Write to the PBMUXSEL to change who is a PWM and who is a GPIO.
	// Write to the OUT to enable/disable low sides.


	PAC55XX_GPIOB->OUT.w = 0;			//PORTB OUT GPIO = 0;
	PAC55XX_SCC->PBMUXSEL.w = 0;		//PORTB PSEL is ALL GPIO's
	//Configure PORTB Outputs to Push Pull
	PAC55XX_GPIOB->MODE.w = MOTOR_OUTPUTS_PUSH_PULL;


	// Configure nIRQ1 interrupt input signals and enable interrupts
	PAC55XX_GPIOA->MODE.P7 = 3;             // Input for over current interrupt
	PAC55XX_GPIOA->INTTYPE.P7 = 0;          // Edge trigger
	PAC55XX_GPIOA->INTCFG.P7 = 0;           // Falling edge
	PAC55XX_GPIOA->INTEDGEBOTH.P7 = 0;      // Only one edge
	PAC55XX_GPIOA->INTCLEAR.P7 = 1;         // Clear any pending PA7 interrupt
	PAC55XX_GPIOA->INTEN.P7 = 1;            // Enable Interrupt
	NVIC_EnableIRQ(GpioA_IRQn);				// Enable interrupts in NVIC

#if BUTTON_CONTROL
	PAC55XX_SCC->PEPUEN.w = 0x03;
#endif


        //HALL_SENSOR_PD012
	//Configure PD0/1/2 for Hall Sensor Inputs
	//PD0 is Hall Sensor U Input
	//PD1 is Hall Sensor V Input
	//PD2 is Hall Sensor W Input
	PAC55XX_GPIOD->MODE.P0 = IO_HIGH_IMPEDENCE_INPUT;
	PAC55XX_GPIOD->MODE.P1 = IO_HIGH_IMPEDENCE_INPUT;
	PAC55XX_GPIOD->MODE.P2 = IO_HIGH_IMPEDENCE_INPUT;

	PAC55XX_SCC->PDMUXSEL.P0 = 2;			//DPM PD0 to TIMERC0
	PAC55XX_SCC->PDMUXSEL.P1 = 2;			//DPM PD1 to TIMERC1
	PAC55XX_SCC->PDMUXSEL.P2 = 2;			//DPM PD2 to TIMERC2

	PAC55XX_SCC->PDPUEN.w = 0x07;			//PD0/1/2 are pulled up

	//Configure TIMERC to act as the Hall Sensor Input Capture Engine
	pac5xxx_timer_clock_config(TimerC, TXCTL_CS_PCLK, TXCTL_PS_DIV16);      // Configure timer clock input for PCLK /16 = 3.125 MHz
	PAC55XX_TIMERC->CTL.BASEIE = 0;
	PAC55XX_TIMERC->CTL.PRDLATCH = TXCTL_PRDLATCH_TXCTR_IM;
	PAC55XX_TIMERC->CCTL0.CCMODE = TXCCTL_CCMODE_CAPTURE;
	PAC55XX_TIMERC->CCTL0.CCINTEDGE = TXCCTL_CCINT_BOTH_EDGES;
	PAC55XX_TIMERC->CCTL0.CCLATCH = TXCCTL_CCLATCH_CAPTURE_BOTH_EDGES;
	PAC55XX_TIMERC->CCTL0.CCINTEN = 1;
	PAC55XX_TIMERC->CCTL1.CCMODE = TXCCTL_CCMODE_CAPTURE;
	PAC55XX_TIMERC->CCTL1.CCINTEDGE = TXCCTL_CCINT_BOTH_EDGES;
	PAC55XX_TIMERC->CCTL1.CCLATCH = TXCCTL_CCLATCH_CAPTURE_BOTH_EDGES;
	PAC55XX_TIMERC->CCTL1.CCINTEN = 1;
	PAC55XX_TIMERC->CCTL2.CCMODE = TXCCTL_CCMODE_CAPTURE;
	PAC55XX_TIMERC->CCTL2.CCINTEDGE = TXCCTL_CCINT_BOTH_EDGES;
	PAC55XX_TIMERC->CCTL2.CCLATCH = TXCCTL_CCLATCH_CAPTURE_BOTH_EDGES;
	PAC55XX_TIMERC->CCTL2.CCINTEN = 1;
	PAC55XX_TIMERC->INT.w = 0x07;
	pac5xxx_timer_base_config(TimerC, 0x7FFF, 0, TxCTL_MODE_UP, 0);			// Configure Timer
	NVIC_SetPriority(TimerC_IRQn, 1);										// Set Priority; Enablement will occur after Open Loop ends; Disablement will occur when motor is disabled
	NVIC_EnableIRQ(TimerC_IRQn);



	timer_d_div = 4;

	//Configure TIMERD to act as the Angle Advance Commutator
	pac5xxx_timer_clock_config(TimerD, TXCTL_CS_PCLK, TXCTL_PS_DIV16);      // Configure timer clock input for PCLK /16 = 3.125 MHz
	pac5xxx_timer_base_config(TimerD, 0x7FFF, 0, TxCTL_MODE_DISABLED, 0);	// Configure Timer
	PAC55XX_TIMERD->CTL.BASEIE = 0;
	PAC55XX_TIMERC->CTL.PRDLATCH = TXCTL_PRDLATCH_TXCTR_IM;
	NVIC_SetPriority(TimerD_IRQn, 1);										// Set Priority; Enablement will occur after Open Loop ends; Disablement will occur when motor is disabled
	NVIC_EnableIRQ(TimerD_IRQn);

	//Configure SysTick Timer
    SysTick->LOAD = 150000;
    SysTick->VAL = 0;
    SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk +  SysTick_CTRL_CLKSOURCE_Msk + SysTick_CTRL_TICKINT_Msk);
	NVIC_SetPriority(SysTick_IRQn, 3);
}

#endif
