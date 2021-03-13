/****************************************************************************
 * 
 *      Support functions for all BLDC Algorithms
 * 
 *
 ******************************************************************************/
#define INCLUDE_EXTERNS
#include "bldc_common.h"

/**
 * @brief  This is the interrupt handler for the SysTick which will be used to generate 1 ms periodic pulses to
 * 			run the main loop state machine.
 *
 * @return none
 *
 */
void SysTick_Handler(void)
{
	millisecond = 1;
}

/**
 * @brief  This is the interrupt handler for GPIOA, which is where the IRQ1 pin
 *         resides.
 *
 *
 * @return none
 */

void GpioA_IRQHandler(void)
{
	if (PAC55XX_GPIOA->INTFLAG.w & NIRQ1_PIN_MASK)
		{
		motor_pwm_disable();
		app_status |= status_over_current;
		PAC55XX_GPIOA->INTCLEAR.P7 = 1;
		PAC55XX_TIMERD->CCTL0.CCINTEN = 0;
		}
}

/**
 * @brief	This function configures dead time.
 *
 * @return none
 *
 */

void Set_Dead_Time(void)
{
	// Configure Hardware Dead time Generator
	pac5xxx_dtg_config2(&(PAC55XX_TIMER_SEL->DTGCTL0), dt_leading_ticks, dt_trailing_ticks);				// Configure DTGA0
	pac5xxx_dtg_config2(&(PAC55XX_TIMER_SEL->DTGCTL1), dt_leading_ticks, dt_trailing_ticks);				// Configure DTGA1
	pac5xxx_dtg_config2(&(PAC55XX_TIMER_SEL->DTGCTL2), dt_leading_ticks, dt_trailing_ticks);				// Configure DTGA2	
}

/**
 * @brief	This function calibrates the differential amplifiers.
 *
 * @return none
 *
 */

void Differential_Amplifier_Calibrate(void)
{
	phase_u_offset = 0;
	phase_v_offset = 0;
	phase_w_offset = 0;
	ADC_Counter = 512;
	ADCSM_State = ADCSM_Calibrate;
}

/**
 * @brief	This function clears Over Current Events and re-enables the power manager.
 *
 * @return none
 *
 */
void oc_reset(void)
{
	uint16_t register_val;
	uint16_t clear_counter;

	// Disable global interrupts until OC reset is complete
	__disable_irq();
	
	clear_counter = 0;
	register_val = pac5xxx_tile_register_read(ADDR_PROTSTAT);
	// Clear int/fault flags
	pac5xxx_tile_register_write(ADDR_PROTINTM, 0x00);
	while ( register_val && clear_counter < 1000)
		{
		clear_counter++;
		register_val = pac5xxx_tile_register_read(ADDR_PROTSTAT);
		pac5xxx_tile_register_write(ADDR_PROTSTAT, PROTINTM_MASK);	
		}

	if (clear_counter < 1000)
		{
		pac5xxx_tile_register_write(ADDR_PROTINTM, PROTINTM_MASK);
		// Disable driver
		do
			{
			pac5xxx_tile_register_write(ADDR_ENDRV, 0);
			register_val = pac5xxx_tile_register_read(ADDR_ENDRV);
			}
		while ((register_val&0x1) != 0x0);

		pac5xxx_tile_register_write(ADDR_ENDRV, 1);

		// Update status
		app_status &= ~(status_motor_enabled + status_over_current + status_motor_stalled);
		}
	else
		{
		app_status |= status_over_current;
		app_status &= ~status_motor_enabled;
		//Over Current Condition remains
		}
	// Enable global interrupts
	__enable_irq();

	// Turn back on ADC and start for control processing. Motor still disabled until UART command.
	pac5xxx_adc_enable(1);
	pac5xxx_adc_start();
}

/**
 * @brief	This function transforms the motor speed in Hz to the respective number of clock cycles required
 * 			The equation on this function is # ticks = PWM_FREQ / (6 * SpeedInHz)
 * 			The PWM_FREQ (units is KHz) is programmable and is computed in the UART module
 *
 * @return 	The number of clock cycles corresponding to the desired frequency
 *
 */

APP_RAMFUNC fix16_t HertzToTicks(fix16_t Hertz, uint32_t Freq)
{
	fix16_t tmp;
	tmp = fix16_mul_new_16_16(6 << 16, Hertz);						// Hz * 6 commutation states
	tmp = fix16_div(Freq, tmp);
	return tmp << 10; 												// Converted to fix16 after multiplying by 1024
}

fix16_t HertzToTicksSine(fix16_t Hertz, uint32_t Freq)
{
	fix16_t tmp;
	tmp = fix16_mul_new_16_16(360 << 16, Hertz);						// Hz * 6 commutation states
	tmp = fix16_div(Freq, tmp);
	return tmp << 10; 												// Converted to fix16 after multiplying by 1024

}

/**
 * @brief	This function gets the ADC conversions for the VIN scaler and the potentiometer
 *
 * @return 	NONE
 *
 */
void check_adc(void)
{

#ifdef PAC5556
	vin_volts = (100 + 100 * (VIN_VOLTS_VAL - vms100)/(vms200 - vms100)) << 16;
#else
	vin_volts = fix16_mul_new_16_16((VIN_VOLTS_VAL << 16), VOLT_DIV_FIX16);
#endif
	pot_volts_command = POT_VOLTS_VAL * pwm_period_ticks / 4092;
}

/**
 * @brief	This function checks the VIN scaled voltage and determines whether there is
 * 			enough battery voltage to maintain motor operational. When battery voltage is below
 * 			threshold, the motor is disabled.
 *
 * @return 	NONE
 *
 */

void check_vbat(void)
{
	if (vin_volts < VIN_VOLTS_LEGAL)
		{
		vin_check_debounce--;
		if (vin_check_debounce <= 0)
			{
			if (app_status & status_motor_enabled)
				{
				motor_pwm_disable();
				}
			app_status |= status_under_voltage;
			}
		}
	else
		{
		vin_check_debounce++;
		if (vin_check_debounce > VIN_CHECK_DEB_MAX)
			{
			vin_check_debounce = VIN_CHECK_DEB_MAX;
			app_status &= ~status_under_voltage;
			}
		}
}

/**
 * @brief	This function checks the state of the Push Buttons at PE4 and PE5.
 * 			In this FW, the PE4 button is used to turn the motor ON and the
 * 			PE5 button is used to turn the motor OFF
 *
 * @return 	NONE
 *
 */


void ButtonControlCheck(void)
{
uint32_t buttonStates = (PAC55XX_GPIOE->IN.w & 0x03);

if (!(buttonStates & 0x01) && !(app_status & status_motor_enabled))
	SMS_State = SMS_Align;
if (!(buttonStates & 0x02) && (app_status & status_motor_enabled))
	motor_pwm_disable();
}

/**
 * @brief	This function configures the H Bridge as a speaker driver and enables the motor
 * 			to apply a beep sound at the frequency of the musical notes denoted from 0 to 127
 *
 * @return 	NONE
 *
 */

void beep_on(int note)
{
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
	ADCSM_State = ADCSM_IRegulate;

	pac5xxx_timer_base_config(TimerD, beep_notes[note], 0, TxCTL_MODE_UP, 0);
	PAC55XX_TIMERD->CTL.BASEIE = 1;									// Enable Timer D Base Interrupt
	NVIC_EnableIRQ(TimerD_IRQn);
}

/**
 * @brief	This function disables the H Bridge and stops the sound function
 *
 * @return 	NONE
 *
 */

void beep_off(void)
{
	#ifdef PAC5556
	PAC55XX_GPIOB->OUT.w = 0;			//PORTB OUT GPIO = 0;
	PAC55XX_SCC->PBMUXSEL.w = 0;		//PORTB PSEL is ALL GPIO's
	PAC55XX_SCC->PCMUXSEL.w = 0;		//PORTC PSEL is ALL GPIO's
	#else
	PAC55XX_GPIOB->OUT.w = 0;			//PORTB OUT GPIO = 0;
	PAC55XX_SCC->PBMUXSEL.w = 0;		//PORTB PSEL is ALL GPIO's
	#endif
	
	app_status &= ~status_motor_enabled;
	PAC55XX_TIMERD->CTL.BASEIE = 0;
	NVIC_DisableIRQ(TimerD_IRQn);
	SMS_State = SMS_Idle;
}

/**
 * @brief	This function commands the device to enter lowest power consumtion mode - Hibernation Mode
 * 			AIO6 is configured to Push Button Mode. A transition from LO to HI exits the Hibernation Mode
 *
 * @return 	NONE
 *
 */

#ifndef CAFE_ARCH2

void mcu_hibernate(void)
{
	int temp_var;

	if (!(app_status & status_motor_enabled))
		{
		temp_var = pac5xxx_tile_register_read(ADDR_PSTATSET);
		temp_var |= 0x01;
		pac5xxx_tile_register_write(ADDR_PSTATSET, temp_var);			//Enable Push Button Mode in AIO6

		pac5xxx_tile_register_write(ADDR_PWRCTL, 0x80);					//Enter Hibernate mode
		}
}

#else
void mcu_hibernate(void)
{

	if (!(app_status & status_motor_enabled))
		{
		//Enable Push Button Mode in AIO6
		module_enable_bits |= 0x40;
		pac5xxx_tile_register_write(ADDR_MISC, module_enable_bits);

		//Enter Hibernate mode
		module_enable_bits |= 0x80;
		pac5xxx_tile_register_write(ADDR_MISC, module_enable_bits);
		}
}
#endif
