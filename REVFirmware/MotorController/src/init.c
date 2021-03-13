/****************************************************************************
*
*       Initialization functions
*
******************************************************************************/
#define INCLUDE_EXTERNS
#include "bldc_common.h"


void app_init(void)
{
	last_sample_stored = 0;						// Last good commutation state sample store
/*      CHECK FUNCTIONALITY

	sample_delay = SAMPLE_DELAY_DEF;
	PAC55XX_TIMER_SEL->CCTR2.CTR = sample_delay;

	closed_loop_speed_hz = START_SPEED_HZ << 16;
	speed_ref_ticks = HertzToTicks(closed_loop_speed_hz, (TIMER_D_FREQ_F16 >> timer_d_div));
	speed_ref_command_hz = START_SPEED_HZ;
	speed_ref_hz = speed_ref_command_hz;

	blanking_cycles = START_BLANKING_CYCLES;
	good_samples = START_GOOD_SAMPLES;

	SMS_Counter = 0;
	SMS_State = SMS_Idle;

	start_iq_ref = START_IQ_REF_DEF;
	align_time_ms = ALIGN_TIME_MS_DEF;
	ol_start_hz = OL_START_HZ_DEF;
	ol_switchover_hz = OL_SWITCHOVER_HZ_DEF;

	ol_accel_period = DEFAULT_OL_ACCEL_PERIOD;
	ol_accel_increase = DEFAULT_OL_ACCEL_INC;

	sine_scale_max = SCALE_PERCENT;

	cl_accel_period = DEFAULT_CL_ACCEL_PERIOD;
	cl_accel_increase = DEFAULT_CL_ACCEL_INC;
	tmp_cl_accel = 0;

	commutation_advanced_rise = ADV_DLY_DEF << 16;
	commutation_advanced_fall = ADV_DLY_DEF << 16;

	commutation_advanced_rise = fix16_mul_new_16_16(commutation_advanced_rise, HALF_DEGREE_ADV_DLY);
	commutation_advanced_fall = fix16_mul_new_16_16(commutation_advanced_fall, HALF_DEGREE_ADV_DLY);

	open_loop = 1;
	ADCSM_State = ADCSM_SineWave;

	vin_check_debounce = VIN_CHECK_DEB_MAX;

	millisecond = 0;

	beep_freq_hz = BEEP_FREQ_HZ_DEF;
	beep_pwm_dc = BEEP_PWM_DC_DEF;

	enable_speed_pi = EN_SPEED_PI;
	if (enable_speed_pi)
		app_status |= status_speed_pi_enabled;
	else
		app_status &= ~status_speed_pi_enabled;

	enable_current_pi = EN_CURRENT_PI;
	if (enable_current_pi)
		app_status |= status_current_pi_enabled;
	else
		app_status &= ~status_current_pi_enabled;

	switchover = EN_SWITCHOVER;
	if (switchover)
		app_status |= status_switchover_enabled;
	else
		app_status &= ~status_switchover_enabled;

	motordir = BLDC_DIRECTION;
	if (motordir)
		app_status |= status_motor_direction;
	else
		app_status &= ~status_motor_direction;

	tmp_current_min = IQ_PI_MIN;
	tmp_current_max = IQ_PI_MAX;


	


//HALL_SENSOR_APP
	app_status &= ~status_firmware_style;





	vm_scaling = VOLT_DIV_FIX16;		//Defined by external resistors at EVK defined PCx pin

	
	false_speed_ticks = HertzToTicks((FALSE_SPEED_HZ), (TIMER_D_FREQ_F16 >> timer_d_div)) >> 16;
*/
}

/**
 * @brief  This function Initializes the PI Loops
 *
 * @return none
 *
 */

void pi_init(void)
{
	//Current to PWM Duty Cycle PI Loop
	iq_pid.Kp = IQ_PI_KP;
	iq_pid.Ki = IQ_PI_KI;
	iq_pid.min_value = fix16_mul_new_16_16(tmp_current_min, pwm_period_ticks << 16);
	iq_pid.PI_sat = iq_pid.min_value;
	iq_pid.I_prev = iq_pid.min_value;
	iq_pid.max_value = fix16_mul_new_16_16(tmp_current_max, pwm_period_ticks << 16);

	iq_pid.Td = fix16_div(IQ_PI_TD, (app_pwm_period << 16));	//1024 / (PWM_FREQ_KHZ * ADC_FREQ)
	iq_pid.Td = fix16_mul_new_16_16(iq_pid.Td, iq_pid.Ki);		//IQ PID TD must include both the TD as well as the KI

	//Speed to Current PI Loop
	speed_pid.Kp = SPEED_PI_KP;
	speed_pid.Ki = SPEED_PI_KI;
	speed_pid.min_value = SPEED_PI_MIN << 16;
	speed_pid.max_value = SPEED_PI_MAX << 16;

	speed_pid.Td = SPEED_PI_TD;
	speed_pid.Td = fix16_mul_new_16_16(speed_pid.Td, speed_pid.Ki);

	//Speed to PWM Duty Cycle PI Loop
	speed_pwm_pid.Kp = FIX16(0.01);
	speed_pwm_pid.Ki = FIX16(0.01);
	speed_pwm_pid.min_value = iq_pid.min_value;
	speed_pwm_pid.max_value = iq_pid.max_value;

	speed_pwm_pid.Td = speed_pid.Td;
}

/**
 * @brief  This function selects which device is being employed (For GUI recognition purposes)
 *
 * @return none
 *
 */
void device_select_init(void)
{

	app_status = PAC5532_Device << 27;

}

/**
 * @brief  This function configures the UART Serial Communications Block
 *
 * @return none
 *
 */
/*


/*
void adc_init(void)
{
	pac5xxx_adc_enable(0);
	// EMUX configuration
	pac5xxx_adc_emux_config(ADCEMUXCTL_DTSE_SEQ, ADCEMUXCTL_EMUXDIV_DIV16);	 		// Configure EMUX to do conversions from ADC sequencer, /2 EMUX divider (HCLK=50MHz/2 = 25MHz)
	pac5xxx_adc_config_emux_io();													// Configure device IO for EMUX

	//ADC configuration
	pac5xxx_adc_config(ADCCTL_MODE_DTSE, ADCCTL_CLKDIV_DIV8, 0);		// Configure ADC for ASC0 independent trigger sequence, /3 divider (PLLCLK=100MHz/3=16.66MHz), repeat mode
	pac5xxx_adc_config_io(ADC_CHANNEL_MASK);										// Configure device IO for ADC conversions (as Analog inputs)
	PAC55XX_GPIOF->MODE.P6 = 0;                     // ADC INPUT
	//AS0 configuration


	//pac5xxx_adc_config_pwm()
	PAC55XX_ADC->DTSETRIGENT0TO3.TRIG2CFGIDX = 0;						//PWMA2 triggers a sequence starting with element 0
	PAC55XX_ADC->DTSETRIGENT0TO3.TRIG2EDGE = ADCDTSE_TRIGEDGE_FALLING;


	//void pac55xx_adc_dtse_element_config(DTSE_SEQ_CFG_TYPEDEF * reg_add, int sel_emuxc, int adc_channel, int delay, int EMUX_data, int NoConv, int Seq_done, int IRQ_number, int IRQ_enable)
	pac5xxx_dtse_seq_config(0, ADCCTL_ADMUX_VIN, ADC_SEQ_HBU_EDATA, 0, 0);	// Convert VIN - Dummy EMUX
	pac5xxx_dtse_seq_config(1, ADCCTL_CHANNEL_ADC0, ADC_SEQ_HBU_EDATA, 0, 0);	// RSU - Dummy EMUX
	pac5xxx_dtse_seq_config(2, ADCCTL_CHANNEL_ADC0, ADC_SEQ_HBV_EDATA, 0, 0);	// Convert RSU
	pac5xxx_dtse_seq_config(3, ADCCTL_CHANNEL_ADC0, ADC_SEQ_HBV_EDATA, 0, 0);	// RSV - Dummy EMUX
	pac5xxx_dtse_seq_config(4, ADCCTL_CHANNEL_ADC0, ADC_SEQ_HBW_EDATA, 0, 0);	// Convert RSV
	pac5xxx_dtse_seq_config(5, ADCCTL_CHANNEL_ADC5, ADC_SEQ_HBW_EDATA, 0, 0);	// RSw - Dummy EMUX - Converrt Potentiometer on PF5
	pac5xxx_dtse_seq_config(6, ADCCTL_CHANNEL_ADC0, ADC_SEQ_VMS_EDATA, ADC_IRQ0_EN, SEQ_END);	// Convert RSW

	// Enable ADC interrupts on AS0 for control loop
	NVIC_SetPriority(ADC0_IRQn, 2);													// Configure interrupt priority
	NVIC_EnableIRQ(ADC0_IRQn);														// Enable ADC interrupts in the NVIC

	// Enable ADC
	pac5xxx_adc_enable(1);
	pac5xxx_adc_start();
}
*/
/**
 * @brief  This function configures PAC analog peripheral
 *
 * @return none
 *
 */


void cafe_init(void)
{
        hp_over_current_limit = HP_OCP_DEF;
	lp_over_current_limit = LP_OCP_DEF;
        
        
  
	// Configure SOC Bridge for talking to MC02
	pac5xxx_tile_socbridge_config(1, 0);						// enable, int_enable

	// Write all CAFE registers
	if (pac5xxx_tile_register_read(ADDR_STATUS))				// If any power manager error bits set on startup, clear them
		{
		//Add desired checks here and then clear
		pac5xxx_tile_register_write(ADDR_STATUS, 0xFF);
		}

	//pac5xxx_tile_register_write(ADDR_PWRCTL, 0x28);				//PWR MON = VMS1
	pac5xxx_tile_register_write(ADDR_PWRCTL, 0x38);				//PWR MON = VMS2

	// Set HPROT protection threshold
	pac5xxx_tile_register_write(ADDR_HPDACH, hp_over_current_limit >> 2);
	pac5xxx_tile_register_write(ADDR_HPDACL, hp_over_current_limit & 0x03);

	// Set LPROT protection threshold
	pac5xxx_tile_register_write(ADDR_LPDACH, lp_over_current_limit >> 2);
	pac5xxx_tile_register_write(ADDR_LPDACL, lp_over_current_limit & 0x03);

	// Enable Module - Must be done before configuring Diff Amp and OCP Protection Scheme
	module_enable_bits = MODULE_ENABLE_FLAGS_DEF;
	pac5xxx_tile_register_write(ADDR_MISC, module_enable_bits);

	// Configure AIOxx for ADC sampling with OC protection
	pac5xxx_tile_register_write(ADDR_CFGAIO0, 0x40 + (DIFFAMP_GAIN_OPT0 << 3) + LPOPT_SEL);			// AIO10: DiffAmp, 16X gain, LPOPT (4 us)
	pac5xxx_tile_register_write(ADDR_CFGAIO1, 0x38 + HPOPT_SEL);			// AIO10: nHP10PR1M set, NO ENOS10, HPOPT (4us)

	pac5xxx_tile_register_write(ADDR_CFGAIO2, 0x40 + (DIFFAMP_GAIN_OPT0 << 3) + LPOPT_SEL);			// AIO32: DiffAmp, 16X gain, LPOPT (4 us)
	pac5xxx_tile_register_write(ADDR_CFGAIO3, 0x38 + HPOPT_SEL);			// AIO32: nHP32PR1M set, NO ENOS32, HPOPT (4us)

	pac5xxx_tile_register_write(ADDR_CFGAIO4, 0x40 + (DIFFAMP_GAIN_OPT0 << 3) + LPOPT_SEL);			// AIO54: DiffAmp, 16X gain, LPOPT (4 us)
	pac5xxx_tile_register_write(ADDR_CFGAIO5, 0x38 + HPOPT_SEL);			// AIO54: nHP54PR1M set, NO ENOS54, HPOPT (4us)

	// Enable protection interrupt mask
	// PROTINTM: nHP54INTM, nHP32INTM, nHP10INTM
	pac5xxx_tile_register_write(ADDR_PROTINTM, PROTINTM_MASK);

        
        /////////////// EDITED - DISABLE OC PROTECT //////////
        
        
	// Disable both HS and LS drivers on PR event (nHSPRM=1, nLSPRM=1);
	// Prop Delay 0 ns; Enable Break Before Make
	pac5xxx_tile_register_write(ADDR_CFGDRV1, 0x01);    
        // originally           pac5xxx_tile_register_write(ADDR_CFGDRV1, 0x0D);   
        
        
        
	// Cycle By Cycle on Diff Amp AIO10/32/54. Disable only high side
	pac5xxx_tile_register_write(ADDR_CFGDRV2, 0x1D);
	// Cycle By Cycle on Diff Amp AIO10/32/54 compared against LPDAC
	pac5xxx_tile_register_write(ADDR_CFGDRV3, 0x54);


	//Configure Blanking Time Feature
	pac5xxx_tile_register_write(ADDR_BLANKING, BLANKING_CONFIG);

	// Set HP comparator hysteresis (HPROTHYS=1b, LPROTHYS=1b)
	pac5xxx_tile_register_write(ADDR_SIGSET, 0x0C);

	//Configure Sensorless Comparators
	pac5xxx_tile_register_write(ADDR_CFGAIO6, 0x26);

	pac5xxx_tile_register_write(ADDR_CFGAIO7, (0xC0 + COMP_POL));			// MODE7[1:0] = 11b (special mode)
	pac5xxx_tile_register_write(ADDR_CFGAIO8, (0xD0 + COMP_POL));			// MODE8[1:0] = 11b (special mode), OPT8[1:0] = 01b (bypass FF, select MUX out for nIRQ2/POS), POL8 = 0 (act high), MUX[2:0] = n/a
	pac5xxx_tile_register_write(ADDR_CFGAIO9, SLCOMP7);			// MODE9[1:0] = 01b (CT Vfilt), OPT9[1:0] = 0bxx (AIO7), POL9 = 0 (act high), MUX[2:0] = n/a

	// HYSMODE[7] = 1b (0/20/40/80), HYSAIO7[3:0] = 1010b (40/40)
	pac5xxx_tile_register_write(ADDR_SPECCFG0, SPECCFG0_CONFIG);
	// HYSAIO8[7:4] = 1010b (40/40), HYSAIO9[3:0] = 1010b (40/40)
	pac5xxx_tile_register_write(ADDR_SPECCFG1, SPECCFG1_CONFIG);

	// SMUXAIO7[3:0] = 01b (AB1 as COMP-), SMUXAIO8[3:0] = 01b (AB1 as COMP-)
	pac5xxx_tile_register_write(ADDR_SPECCFG2, 0x11);
	// SMUXAIO9[3:0] = 01b (AB1 as COMP-)
	pac5xxx_tile_register_write(ADDR_SPECCFG3, 0x10);

	// EMUXEN=1 to enable EMUX, ENADCBUF=1 for ADC Buffer
	pac5xxx_tile_register_write(ADDR_SHCFG1, 0x00);
	pac5xxx_tile_register_write(ADDR_SHCFG1, 0x18);

	// Enable Drivers
	pac5xxx_tile_register_write(ADDR_ENDRV, 0x00);
	pac5xxx_tile_register_write(ADDR_ENDRV, 0x01);
}

void motor_params_init(void){
  
  current_speed = 1;
  motor_dir = 0;
  acceleration_factor = 10;
  motor_status = motor_disabled;
  
  
}
