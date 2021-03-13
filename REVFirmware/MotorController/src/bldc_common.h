/****************************************************************************
 * 
 *     Public interface to BLDC Common Application
 * 
 *
 * 
 ******************************************************************************/

#ifndef BLDC_COMMON_H
#define BLDC_COMMON_H

#ifndef INCLUDE_EXTERNS
#define	EXTERN	volatile
#else
#define	EXTERN	extern	volatile
#endif

#define         HALL_SENSOR
#ifdef 		HALL_SENSOR
#define		HALL_SENSOR_APP
#endif

#define PAC5532EVK1
#ifdef	PAC5532EVK1
#define	PAC5532
#define	CAFE_ARCH2
#define	ADCCTL_ADMUX_VIN	ADCCTL_ADMUX_AD3
#define RSENSE_mOHMS		10
#define VIN_SCL_RTOP		560
#define VIN_SCL_RBOT		10
#define	HALL_SENSOR_PD012	1
#define	HALL_SENSOR_PC456	0
#define	HALL_SENSOR_PD456	0
#define	HALL_SENSOR_PE456	0
#define SIGSET_DEF			0x0C
#define HP_IOCP_AMPS		31					//Desired OCP Current in Amps
#define LP_IOCP_AMPS		31					//Desired OCP Current in Amps
#endif



#include "pac5xxx.h"
#include "pac5xxx_tile_driver_manager.h"
#include "pac5xxx_tile_power_manager.h"
#include "pac5xxx_tile_signal_manager.h"
#include "pac5xxx_tile_system_manager.h"

#include "pac5xxx_driver_adc.h"
#include "pac5xxx_driver_timer.h"
#include "pac5xxx_driver_socbridge.h"
#include "pac5xxx_driver_tile.h"
#include "pac5xxx_driver_system.h"
#include "pac5xxx_driver_memory.h"
#include "pac5xxx_driver_uart.h"
#include "uart.h"
#include "fix16.h"
#include "version.h"
#include "pid.h"
#include "motordef.h"
#include "state_machine.h"




#define FIX16(n)	((long)(65536.0 * n ))

#define EN_SPEED_PI			0							// 1 to enable Speed PI Loop
#define EN_CURRENT_PI		0							// 1 to enable Current PI Loop
#define EN_SWITCHOVER		0							// 1 to enable crossing from Open Loop to closed Loop (BEMF)
#define	STALL_DETECT		0
#define	NODC				0							// 0 for BUCK or SEPIC, 1 for No Switching Regulation
#define	TUNING				0							// For DEBUG
#define	UPDATE_DEBUG		0							// 1 for running the Update Debug functionality
#define	POT_CONTROL			0							// 1 for using potentiometer control
#define	COMM_SUPPORT		1							// 1 to include UART Support, 0 to remove from code space
#define BUTTON_CONTROL		0							//1 to start motor from Push Button
#define	AUTO_START			0							//1 to start motor automatically after reset
#define	CHECK_VBAT			0
#define ANGLE_ADVANCE 		0							//1 to enable Angle Advance (both BEMF and HS)
#define	DUAL_AA 			0							//1 to enable different Angle Advance for Rise and Falling edges
#define	CODELESS_HS			0							//1 to enable Code-less Hall Sensor Commutation

//#define APP_RAMFUNC_LINK
#ifdef APP_RAMFUNC_LINK
#define APP_RAMFUNC PAC5XXX_RAMFUNC
#else
#define APP_RAMFUNC
#endif

//#define ADC_CHANNEL_MASK	((1L << ADC0) | (1L << ADC2) | (1L << ADC3) | (1L << ADC4))	// Mask of ADC channels to perform conversions for

#if POT_CONTROL
#define ADC_CHANNEL_MASK	(1L << ADC0) + (1L << ADC5) 							// Mask of ADC channels to perform conversions for
#else
#define ADC_CHANNEL_MASK	(1L << ADC0) 											// Mask of ADC channels to perform conversions for
#endif

#define	ADC_SEQ_EDATA		SIGMGR_MSPI_SH_COMP										// Latch for the comparator output (SH1 on EMUX) Selecting AD0 on ADCMux, although irrelevant

#define ADC_SEQ_VIN				0													// AS0 sequence number for VIN
#define	ADC_SEQ_U1				1													// AS0 sequence number for H Bridge Current (HBI)
#define	ADC_SEQ_V1				2													// AS0 sequence number for H Bridge Current (HBI)
#define	ADC_SEQ_W1				3													// AS0 sequence number for H Bridge Current (HBI)
#define	ADC_SEQ_VPOT			4													// AS0 sequence number for H Bridge Current (HBI)

#define	VIN_VOLTS_VAL			PAC55XX_ADC->DTSERES0.VAL
#define	U1_ADC_VAL				PAC55XX_ADC->DTSERES2.VAL
#define	V1_ADC_VAL				PAC55XX_ADC->DTSERES4.VAL
#define	W1_ADC_VAL				PAC55XX_ADC->DTSERES6.VAL
#define	POT_VOLTS_VAL			PAC55XX_ADC->DTSERES5.VAL

#define	ADC_SEQ_HBU_EDATA		EMUX_AIO10
#define	ADC_SEQ_HBV_EDATA		EMUX_AIO32
#define	ADC_SEQ_HBW_EDATA		EMUX_AIO54
#define	ADC_SEQ_VMS_EDATA		SIGMGR_AB11

typedef enum
{
	TimerC_idle = 0,
	TimerC_Align_And_Go,
	TimerC_getslcomp_first_sample,
	TimerC_getslcomp_samples,
	TimerC_getslcomp_commutation_wait,
	TimerC_getslcomp_blanking_cycles
}TimerC_States;

typedef enum
{
	TimerD_Idle = 0,
	TimerD_SineWaveOL,
	TimerD_SixStepOL,
	TimerD_Switchover
}TimerD_States;

typedef enum
{
	ADCSM_Idle = 0,
	ADCSM_Calibrate,
	ADCSM_SineWave,
	ADCSM_SixStep,
	ADCSM_IRegulate
}ADCSM_States;

typedef enum
{
	status_motor_enabled = 1,
	status_over_current = 2,
	status_motor_stalled = 4,
	status_closed_loop = 8,
	status_under_voltage = 16,
	status_crc_test_check = 32,
	status_motor_direction = 64,
	status_speed_pi_enabled = 128,
	status_current_pi_enabled = 256,
	status_switchover_enabled = 512,
	status_firmware_style = 1024
} StatusStateBits;

typedef enum
{
	PAC5210_Device = 0,
	PAC5220_Device,
	PAC5222_Device,
	PAC5223_Device,
	PAC5225_Device,
	PAC5232_Device,
	PAC5250_Device,
	PAC5253_Device,
	PAC5255_Device,
	PAC5256_Device,
	PAC5523_Device,					//10
	PAC5524_Device,
	PAC5527_Device,
	PAC5532_Device,
	PAC5556_Device					//14
}DeviceSelect;

typedef enum
{
	pwmc0 = 0,
	pwmc1 = 1,
	pwmc2 = 2,
	pwmc4 = 3,
	pwmc5 = 4,
	pwmc6 = 5,
	pwmc_base = 6,
	firstcomm = 7
} CCRNumbers;

// System's Definitions

#define	TIMER_X_FREQ_CNV			100000		//Applies to both TIMERA and TIMERB depending on PAC55xx device being employed


#define	MOTOR_OUT_PORTB_ALL_PWMS	0x01110111
#define	MOTOR_OUT_PORTB_ALL_GPIO	0
#define	MOTOR_OUTPUTS_PUSH_PULL		0x1515
#define PAC55XX_TIMER_SEL			PAC55XX_TIMERA


#define DT_LED_TICKS			50
#define DT_TED_TICKS			50
#define	SAMPLE_DELAY_DEF		150
#define NIRQ1_PIN_MASK			0x80
#define NIRQ2_PIN_MASK			0x01
#define	STALL_DETECT_DEF		25000
#define FALSE_SPEED_HZ			FIX16(3000)
#define	AVG_SMP_CNT				6
#define	AVRG_DIV				FIX16(1/AVG_SMP_CNT)
#define	ONEDIV60				FIX16(1/60)
#define ONE_SIXTH_F16			FIX16(1/6)
#define	DEBUG_MAX				60

#ifndef CAFE_ARCH2
	#define	COMP_POL			0x00
#else
	#define	COMP_POL			0x08
#endif

#define SLCOMP7					0xD1 + COMP_POL				//AIO7 - PHASE U to Comparator POS
#define SLCOMP8					0xE1 + COMP_POL				//AIO8 - PHASE V to Comparator POS
#define SLCOMP9					0xF1 + COMP_POL	 			//AIO9 - PHASE W to Comparator POS

//***********************************************************************
// TIMER D Dividing Factor
// TIMERD_FREQ = 50 MHz
// On Sensorless Application, the TIMERD Divider and Period are controlled from the GUI.
// On Hall Sensor Application, the TIMERD Divider and Period are defined below
#define	TIMERD_FREQ_MHz			50000000
#define	TIMER_D_FREQ_F16		0xBEBC2000				//TMRD Freq = 50MHz (This number is divided by 1024 so it can be represented by a Fix16 type)
#define	TIMERD_PERIOD			0x7FFF					//Hall Sensor Only - Ensures speed measurement is a positive number.
#define	TIMER_D_DIVIDER			TxCTL_PS_DIV32			//Hall Sensor Only - A divider of 32, makes the smallest measurable speed, 7 Hz.

//***********************************************************************
// Input Voltage Scaling (VIN_SCALE)
// Scale Factor = 2.5 * (RTOP + RBOT) / (RBOT * 1023)
//#define VIN_SCL_RTOP			//Defined in bldc_hw_select.h - Modify if manually changed from default build
//#define VIN_SCL_RBOT			//Defined in bldc_hw_select.h - Modify if manually changed from default build
#define VOLT_DIV_FIX16			FIX16((VIN_SCL_RTOP + VIN_SCL_RBOT) * 2.5 / (VIN_SCL_RBOT * 1023))

//***********************************************************************
// VIN Checking - if VIN readings fall below VIN_VOLTS_LEGAL motor is disabled
// and marked as stalled
#define	VIN_VOLTS_LEGAL			FIX16(12.2)
#define	VIN_CHECK_DEB_MAX		100

//***********************************************************************
// Potentiometer Driving - if using a potentiometer to drive motor speed
#define	TURN_ON_THRESHOLD_MIN	60
#define	TURN_ON_THRESHOLD_MAX	100
#define	MAX_HZ					512

//***********************************************************************
// BEEP ON Defines
#define BEEP_FREQ_HZ_DEF		1000
#define BEEP_PWM_DC_DEF			10

//***********************************************************************
// Brake Speed
#define	BRAKE_SPEED_HZ			50
#define SPEED_REF_TICKS_TH		(3125000 / (6*BRAKE_SPEED_HZ))

//***********************************************************************
// Differential Amplifier Gain Setting
// Only one item can be chosen

//#define	DIFF_AMP_GAIN_X_0		1
//#define	DIFF_AMP_GAIN_X_1		1
//#define	DIFF_AMP_GAIN_X_2		2
#define	DIFF_AMP_GAIN_X_3		4
//#define	DIFF_AMP_GAIN_X_4		8
//#define	DIFF_AMP_GAIN_X_5		16
//#define	DIFF_AMP_GAIN_X_6		32
//#define	DIFF_AMP_GAIN_X_7		48

#if DIFF_AMP_GAIN_X_0
	#define DIFFAMP_GAIN_X				1
	#define	DIFFAMP_GAIN_OPT0			0
#elif DIFF_AMP_GAIN_X_1
	#define DIFFAMP_GAIN_X				1
	#define	DIFFAMP_GAIN_OPT0			1
#elif DIFF_AMP_GAIN_X_2
	#define DIFFAMP_GAIN_X				2
	#define	DIFFAMP_GAIN_OPT0			2
#elif DIFF_AMP_GAIN_X_3
	#define DIFFAMP_GAIN_X				4
	#define	DIFFAMP_GAIN_OPT0			3
#elif DIFF_AMP_GAIN_X_4
	#define DIFFAMP_GAIN_X				8
	#define	DIFFAMP_GAIN_OPT0			4
#elif DIFF_AMP_GAIN_X_5
	#define DIFFAMP_GAIN_X				16
	#define	DIFFAMP_GAIN_OPT0			5
#elif DIFF_AMP_GAIN_X_6
	#define DIFFAMP_GAIN_X				32
	#define	DIFFAMP_GAIN_OPT0			6
#elif DIFF_AMP_GAIN_X_7
	#define DIFFAMP_GAIN_X				48
	#define	DIFFAMP_GAIN_OPT0			7
#endif

#if defined (PAC5523) || defined (PAC5524) || defined (PAC5556)
//***********************************************************************
// HP/LP Over Current Protection (OCP) Defines
//#define HP_IOCP_AMPS			xx					//Defined on bldc_hw_select.h
//#define LP_IOCP_AMPS			xx					//Defined on bldc_hw_select.h
//#define RSENSE_mOHMS			xx					//Defined on bldc_hw_select.h
#define	HP_OCP_DEF				(HP_IOCP_AMPS * 255 * RSENSE_mOHMS) / (2.5 * 1000)
#define	LP_OCP_DEF				511 + (LP_IOCP_AMPS * DIFFAMP_GAIN_X * 511 * RSENSE_mOHMS) / (1.25 * 1000)

#elif defined (PAC5527) || defined (PAC5532)
//***********************************************************************
// HP/LP Over Current Protection (OCP) Defines
//#define HP_IOCP_AMPS			xx					//Defined on bldc_hw_select.h
//#define LP_IOCP_AMPS			xx					//Defined on bldc_hw_select.h
//#define RSENSE_mOHMS			xx					//Defined on bldc_hw_select.h
#define	HP_OCP_DEF				511 + (HP_IOCP_AMPS * DIFFAMP_GAIN_X * 511 * RSENSE_mOHMS) / (1.25 * 1000)
#define	LP_OCP_DEF				511 + (LP_IOCP_AMPS * DIFFAMP_GAIN_X * 511 * RSENSE_mOHMS) / (1.25 * 1000)
#endif

//***********************************************************************
// HPOPT and LPOPT
#define	xPOPT_DIS				0
#define	xPOPT_1US				1
#define	xPOPT_2US				2
#define	xPOPT_4US				3

#define	LPOPT_SEL				xPOPT_4US
#define	HPOPT_SEL				xPOPT_4US
//***********************************************************************
// Protection Interrupt Mask PROTINTM REGISTER

#define	NHP54INTM_MASK			1
#define	NHP32INTM_MASK			1
#define	NHP10INTM_MASK			1
#define	NLP54INTM_MASK			1
#define	NLP32INTM_MASK			1
#define	NLP10INTM_MASK			1

#define	PROTINTM_MASK			((NHP54INTM_MASK << 6)	+ (NHP32INTM_MASK << 5) + (NHP10INTM_MASK << 4) + (NLP54INTM_MASK << 2) + (NLP32INTM_MASK << 1) + (NLP10INTM_MASK << 0))

//***********************************************************************
// MODULE ENABLE CAFE REGISTER
#define	HIB_BIT					0					//0 = Normal Shutdown Mode; 1 = HIbernate Shutdown Mode
#define PBEN_BIT				0					//0 = Push Button Disabled; 1 = Push Button Enabled (AIO6)
#define	VREFSET_BIT				0					//0 = ADC VREF is 2.5V; 1 = ADC VREF is 3.0V
#define	CLKOUTEN_BIT			0					//0 = Disabled; 1 = Enabled
#define	TPBD					0					//0 = 32 ms; 1 = 1 ms Push Button Deglitch Time
#define	MCUALIVE_BIT			1					//1
#define	ENSIG_BIT				1					//0 = Disabled; 1 = Enabled
#define MODULE_ENABLE_FLAGS_DEF		(HIB_BIT << 7) + (PBEN_BIT << 6) + (VREFSET_BIT << 5) + (CLKOUTEN_BIT << 4) + (MCUALIVE_BIT << 3) + (TPBD << 2) + (ENSIG_BIT << 0)

//***********************************************************************
// BLANKING CAFE REGISTER
#define	BLANKING_100ns			0x00
#define	BLANKING_250ns			0x01
#define	BLANKING_500ns			0x02
#define	BLANKING_750ns			0x03
#define	BLANKING_1000ns			0x04
#define	BLANKING_1250ns			0x05
#define	BLANKING_1500ns			0x06
#define	BLANKING_2000ns			0x07
#define	BLANKING_2500ns			0x08
#define	BLANKING_3000ns			0x09
#define	BLANKING_3500ns			0x0A
#define	BLANKING_4000ns			0x0B
#define	BLANKING_4500ns			0x0C
#define	BLANKING_5000ns			0x0D
#define	BLANKING_5500ns			0x0E
#define	BLANKING_6000ns			0x0F

#define	BLANKING_DIS			0x00
#define	BLANKING_LEAD			0x01
#define	BLANKING_TRAIL			0x02
#define	BLANKING_BOTH			0x03

#ifdef CAFE_ARCH2
#define	BLANKING_CONFIG			(BLANKING_1000ns << 4) + BLANKING_BOTH
#else
#define	BLANK_TIME				BLANKING_100ns
#endif

//***********************************************************************
// HYSTERESIS CAFE REGISTER

#define	HYSMODE_0				0x00
#define	HYSMODE_1				0x80

#define	HYSAIO_0_R0_F0			0x00
#define	HYSAIO_0_R0_F5			0x01
#define	HYSAIO_0_R0_F10			0x02
#define	HYSAIO_0_R0_F20			0x03
#define	HYSAIO_0_R5_F0			0x04
#define	HYSAIO_0_R5_F5			0x05
#define	HYSAIO_0_R5_F10			0x06
#define	HYSAIO_0_R5_F20			0x07
#define	HYSAIO_0_R10_F0			0x08
#define	HYSAIO_0_R10_F5			0x09
#define	HYSAIO_0_R10_F10		0x0A
#define	HYSAIO_0_R10_F20		0x0B
#define	HYSAIO_0_R20_F0			0x0C
#define	HYSAIO_0_R20_F5			0x0D
#define	HYSAIO_0_R20_F10		0x0E
#define	HYSAIO_0_R20_F20		0x0F

#define	HYSAIO_1_R0_F0			0x00
#define	HYSAIO_1_R0_F20			0x01
#define	HYSAIO_1_R0_F40			0x02
#define	HYSAIO_1_R0_F80			0x03
#define	HYSAIO_1_R20_F0			0x04
#define	HYSAIO_1_R20_F20		0x05
#define	HYSAIO_1_R20_F40		0x06
#define	HYSAIO_1_R20_F80		0x07
#define	HYSAIO_1_R40_F0			0x08
#define	HYSAIO_1_R40_F20		0x09
#define	HYSAIO_1_R40_F40		0x0A
#define	HYSAIO_1_R40_F80		0x0B
#define	HYSAIO_1_R80_F0			0x0C
#define	HYSAIO_1_R80_F20		0x0D
#define	HYSAIO_1_R80_F40		0x0E
#define	HYSAIO_1_R80_F80		0x0F

#define	HYSAIO_SEL				HYSAIO_0_R5_F5

#ifdef CAFE_ARCH2
#define	SPECCFG0_CONFIG			HYSMODE_0 + HYSAIO_SEL
#define	SPECCFG1_CONFIG			(HYSAIO_SEL << 4) + HYSAIO_SEL
#else
#define	SPECCFG1_CONFIG			(HYSAIO_SEL << 4) + HYSAIO_SEL
#define	SPECCFG2_CONFIG			(HYSAIO_SEL << 4) + BLANK_TIME
#endif

//***********************************************************************
// Smart Gate Driver Controls
#define TON_SET_4P7US			0x00
#define TON_SET_2P5US			0x01
#define TON_SET_1P8US			0x02
#define TON_SET_1P3US			0x03
#define TON_SET_1P1US			0x04
#define TON_SET_0P9US			0x05
#define TON_SET_P75US			0x06
#define TON_SET_P55US			0x07

#define LS_TON_SET_DEF			TON_SET_P55US	
#define HS_TON_SET_DEF			TON_SET_P55US

#define	SINK_SOURCE_250MA		0x00
#define	SINK_SOURCE_350MA		0x01
#define	SINK_SOURCE_450MA		0x02
#define	SINK_SOURCE_550MA		0x03
#define	SINK_SOURCE_650MA		0x04
#define	SINK_SOURCE_750MA		0x05
#define	SINK_SOURCE_850MA		0x06
#define	SINK_SOURCE_1000MA		0x07

#define LS_SINK_DEF				SINK_SOURCE_550MA					
#define HS_SINK_DEF				SINK_SOURCE_550MA
#define LS_SOURCE_DEF			SINK_SOURCE_550MA
#define HS_SOURCE_DEF			SINK_SOURCE_550MA

//***********************************************************************
// Debug Quick Functions
#define	DEBUG_E0_R		PAC5XXX_GPIOE->OUT.b |= 0x01;
#define	DEBUG_E0_F		PAC5XXX_GPIOE->OUT.b &= ~0x01;

#define	DEBUG_E3_R		PAC5XXX_GPIOE->OUT.b |= 0x08;
#define	DEBUG_E3_F		PAC5XXX_GPIOE->OUT.b &= ~0x08;

#define	DEBUG_E4_R		PAC5XXX_GPIOE->OUT.b |= 0x10;
#define	DEBUG_E4_F		PAC5XXX_GPIOE->OUT.b &= ~0x10;

#define	DEBUG_E5_R		PAC5XXX_GPIOE->OUT.b |= 0x20;
#define	DEBUG_E5_F		PAC5XXX_GPIOE->OUT.b &= ~0x20;

#define	OPTIMIZE_O0		__attribute__((optimize("O0")))
#define	OPTIMIZE_O1		__attribute__((optimize("O1")))
#define	OPTIMIZE_O2		__attribute__((optimize("O2")))
#define	OPTIMIZE_O3		__attribute__((optimize("O3")))
#define	OPTIMIZE_Os		__attribute__((optimize("Os")))

// Function Prototypes
void check_vbat(void);
void check_adc(void);
void motor_pwm_disable(void);
void app_init(void);
void peripheral_init(void);
void cafe_init(void);
void adc_init(void);
void UART_init(void);
void Set_Dead_Time(void);

void oc_reset(void);
void pi_init(void);
void beep_on(int note);
void beep_off(void);
void state_machine(void);
void mcu_hibernate(void);
void device_select_init(void);
void Differential_Amplifier_Calibrate(void);
fix16_t HertzToTicks(fix16_t Hertz, uint32_t Freq);
fix16_t HertzToTicksSine(fix16_t Hertz, uint32_t Freq);

////////////////           NEW PREPROCESSOR DEFINES                  ///////////////


#define ACCEL_FACTOR 5U









////////////////////////////             NEW FUNCS                  ///////////////

void configure_timer_b_compare_mode(void);
void motor_params_init(void);
void accelerate(void);

///////////////////////////////         NEW DECLARATIONS            //////////////////////////////

//EXTERN uint32_t spi_data;

typedef struct      
{
   /*
    union {
        __IO uint32_t w;
        struct {
            __IO uint32_t SPEED         : 8;    // 7:0      Motor Speed Signal
            __IO uint32_t DIRECTION     : 2;    // 9:8      Motor Direction
            __IO uint32_t BRAKING       : 2;    // 11:10    Motor Brake Signal
            __IO uint32_t DISABLE       : 2;    // 13:12    Disable Motor
                 uint32_t               : 18;   // 32:14
        };
    } DATA_RX;*/                //ACTUAL ONE ABOVE -- BELOW IS FOR ARDUINO COMPATIBLE 8-bit DATA BLOCKS
   union {
        __IO uint32_t w;
        struct {
            __IO uint32_t SPEED         : 5;    // 4:0      Motor Speed Signal
            __IO uint32_t DIRECTION     : 1;    // 5        Motor Direction
            __IO uint32_t BRAKING       : 1;    // 6      Motor Brake Signal
            __IO uint32_t DISABLE       : 1;    // 7    Disable Motor
                 uint32_t               : 24;   // 32:8
        };
    } DATA_RX;
    
    union {
        __IO uint32_t w;
        struct {
            __IO uint32_t RPM           : 8;    // 7:0      Motor RPM
            __IO uint32_t DIRECTION     : 2;    // 6:5      Motor Direction
            __IO uint32_t BRAKING       : 2;    // 7:8      Motor Brake Signal
            __IO uint32_t DISABLE       : 2;    // 9:10     Disable Motor
                 uint32_t               : 21;   // 11:14
        };
    } DATA_TX;

} PAC55XX_SPI_TYPEDEF;



EXTERN PAC55XX_SPI_TYPEDEF SPI;

EXTERN uint32_t motor_pwm_duty;
EXTERN uint32_t motor_dir;
EXTERN uint32_t braking;

EXTERN uint32_t disable_motor;


EXTERN uint32_t motor_speed_scaler;
EXTERN uint32_t current_speed;
EXTERN uint32_t acceleration_factor;

typedef enum
{
  motor_disabled = 0,
  motor_active,
  motor_low_rpm
  
  
}Motor_Status;

EXTERN uint32_t motor_status;




//////////////////////////////////////////////////////////////////////////////////////////////
#if defined HALL_SENSOR_APP
void commutate(uint32_t ccrn);
void aa_commutate(uint32_t ccrn);
#endif

EXTERN PID_Data_Type iq_pid;
EXTERN PID_Data_Type speed_pid;
EXTERN PID_Data_Type speed_pwm_pid;					//PI Loop regulating speed; Input = Measured Speed; Output = PWM Duty Cycle in TIMERA Ticks

#ifdef HALL_SENSOR_APP
EXTERN uint32_t hall_sensor_value;
EXTERN uint32_t next_commutation_state;
#endif

EXTERN uint32_t pwm_duty;
EXTERN uint32_t app_pwm_period;
EXTERN uint32_t pwm_period_ticks;					// Number of timer A ticks for PWM period
EXTERN uint32_t dt_leading_ticks, dt_trailing_ticks;
EXTERN uint32_t app_status;
EXTERN uint32_t hp_over_current_limit;
EXTERN uint16_t lp_over_current_limit;
EXTERN uint32_t millisecond;
EXTERN uint32_t timer_d_div;

EXTERN uint32_t SMS_State;
EXTERN uint32_t SMS_Counter;

EXTERN uint32_t TMRD_State;

EXTERN uint32_t ADCSM_State;
EXTERN uint32_t ADC_Counter;
EXTERN uint32_t single_shunt_current;
EXTERN uint32_t sample_delay;
EXTERN uint32_t phase_u_offset;
EXTERN uint32_t phase_v_offset;
EXTERN uint32_t phase_w_offset;
EXTERN uint32_t pot_volts_command;

EXTERN uint32_t timer_d_latch_in;
EXTERN uint32_t timer_d_offset;
EXTERN uint32_t sample;
EXTERN uint32_t last_sample_stored;
EXTERN uint32_t samplecounter;
EXTERN uint32_t getslcomp_state;
EXTERN uint32_t good_samples;
EXTERN uint32_t blanking_cycles, tmp_blanking_cycles;
EXTERN uint32_t commutation_time;
EXTERN uint32_t sl_current_state;
EXTERN uint32_t stall_counter;
EXTERN uint32_t false_speed_counter;
EXTERN uint32_t false_speed_ticks;

EXTERN uint32_t enable_speed_pi;
EXTERN uint32_t enable_current_pi, tmp_enable_current_pi;
EXTERN uint32_t switchover;
EXTERN uint32_t motordir;

EXTERN uint32_t start_iq_ref;
EXTERN uint32_t align_time_ms;
EXTERN uint32_t ol_start_hz;
EXTERN uint32_t ol_switchover_hz;
EXTERN uint32_t ol_accel_period, ol_accel_increase;
EXTERN uint32_t cl_accel_period, tmp_cl_accel, cl_accel_increase;

EXTERN uint32_t speed_ref_hz, speed_ref_command_hz;
EXTERN uint32_t motorspeed;
EXTERN uint32_t avg_speed_index;
EXTERN uint32_t avg_speed_array[AVG_SMP_CNT];

EXTERN uint32_t sine_index;
EXTERN uint32_t open_loop;
EXTERN uint32_t vin_check_debounce;

EXTERN uint32_t beep_freq_hz;
EXTERN uint32_t beep_pwm_dc;
EXTERN uint32_t tmp_pi_debug_index, pi_debug_index;
EXTERN uint32_t diag_message_offset, diag_note_offset;
EXTERN uint32_t debug_index;
EXTERN uint32_t debug_1;
EXTERN uint32_t debug_2;
EXTERN uint32_t debug_3;
EXTERN uint32_t debug_4;

#if defined (PAC5527)
EXTERN uint32_t ls_sink;
EXTERN uint32_t hs_sink;
EXTERN uint32_t ls_source;
EXTERN uint32_t hs_source;
EXTERN uint32_t ls_ton_set;
EXTERN uint32_t hs_ton_set;
#endif

EXTERN uint32_t sinkls;
EXTERN uint32_t sinkhs;
EXTERN uint32_t sourcels;
EXTERN uint32_t sourcehs;

#ifdef CAFE_ARCH2
EXTERN uint32_t module_enable_bits;
#endif

EXTERN int32_t CRC_Test_Check;

EXTERN fix16_t vm_scaling;
EXTERN fix16_t debug_array[DEBUG_MAX];
EXTERN fix16_t vin_volts;
EXTERN fix16_t pot_volts;
EXTERN fix16_t avg_speed;
EXTERN fix16_t pi_debug_value[2][5];
EXTERN fix16_t iq_ref;
EXTERN fix16_t speed_ref_ticks;
EXTERN fix16_t closed_loop_speed_hz;
EXTERN fix16_t commutation_advanced_rise;
EXTERN fix16_t commutation_advanced_fall;
EXTERN fix16_t stall_speed_hz;
EXTERN fix16_t sine_scale;
EXTERN fix16_t sine_scale_max;
EXTERN fix16_t ol_iqref_increase;	//previously sine_scale_increase
EXTERN fix16_t tmp_current_min, tmp_current_max;
#ifdef PAC5556
EXTERN fix16_t vms100;
EXTERN fix16_t vms200;
#endif

extern const uint16_t beep_notes[128];
extern const uint8_t diag_tunes[4][4];
#if defined SENSORLESS_APP || defined RC_PWM_THROTTLE_APP
	extern const fix16_t sine_wave_3phase[360][3];
#endif

// BLDC DIRECTION 0 LOGIC
//STATE TABLE		PWM-HI			PWM-LO			LO-SIDE		CurrentFlow		TriState	Comparator	Floating Phase	Polarity
//0					PWM4-A3(UH)		PWM0-A0(UL)		A2(WL)		U->W			V			SLCOMP8		V-Falling		1
//1					PWM5-A4(VH)		PWM1-A1((VL)	A2(WL)		V->W			U			SLCOMP7		U-Rising		0
//2					PWM5-A4(VH)		PWM1-A1((VL)	A0(UL)		V->U			W			SLCOMP9		W-Falling		1
//3					PWM6-A5(WH)		PWM2-A2(WL)		A0(UL)		W->U			V			SLCOMP8		V-Rising		0
//4					PWM6-A5(WH)		PWM2-A2(WL)		A1(VL)		W->V			U			SLCOMP7		U-Falling		1
//5					PWM4-A3(UH)		PWM0-A0(UL)		A1(VL)		U->V			W			SLCOMP9		W-Rising		0

// BLDC DIRECTION 1 LOGIC
//STATE TABLE		PWM-HI			PWM-LO			LO-SIDE		CurrentFlow		TriState	Comparator	Floating Phase	Polarity
//0					PWM4-A3(UH)		PWM0-A0(UL)		A2(WL)		U->W			V			SLCOMP8		V-Falling		1
//1					PWM4-A3(UH)		PWM0-A0(UL)		A1(VL)		U->V			W			SLCOMP9		W-Rising		0
//2					PWM6-A5(WH)		PWM2-A2(WL)		A1(VL)		W->V			U			SLCOMP7		U-Falling		1
//3					PWM6-A5(WH)		PWM2-A2(WL)		A0(UL)		W->U			V			SLCOMP8		V-Rising		0
//4					PWM5-A4(VH)		PWM1-A1((VL)	A0(UL)		V->U			W			SLCOMP9		W-Falling		1
//5					PWM5-A4(VH)		PWM1-A1((VL)	A2(WL)		V->W			U			SLCOMP7		U-Rising		0

#ifndef INCLUDE_EXTERNS
	EXTERN const uint32_t psel_mask[2][6] = 			{{0x00010001, 0x00100010, 0x00100010, 0x01000100, 0x01000100, 0x00010001},{0x00010001, 0x00010001, 0x01000100, 0x01000100, 0x00100010, 0x00100010}};

	EXTERN const uint32_t psel_mask_pcmux[2][6] = 		{{0x00010000, 0x00100000, 0x00100000, 0x01000000, 0x01000000, 0x00010000},{0x00010000, 0x00010000, 0x01000000, 0x01000000, 0x00100000, 0x00100000}};
	EXTERN const uint32_t psel_mask_pbmux[2][6] = 		{{0x00000002, 0x00000020, 0x00000020, 0x00000200, 0x00000200, 0x00000002},{0x00000002, 0x00000002, 0x00000200, 0x00000200, 0x00000020, 0x00000020}};


	EXTERN const uint8_t c_pwm_io_state[2][6] = 		{{0x04, 0x04, 0x01, 0x01, 0x02, 0x02},{0x04, 0x02, 0x02, 0x01, 0x01, 0x04}};
	EXTERN const uint8_t slcomp_mux[2][6] = 			{{SLCOMP8, SLCOMP7, SLCOMP9, SLCOMP8, SLCOMP7, SLCOMP9},{SLCOMP8, SLCOMP9, SLCOMP7, SLCOMP8, SLCOMP9, SLCOMP7}};
	EXTERN const uint32_t slcomp_cross_polarity[2][6] = {{0x01, 0, 0x01, 0, 0x01, 0},{0, 0x01, 0, 0x01, 0, 0x01}};

	//EXTERN const uint8_t hs_to_commutation[2][8] = 	{{0x03,0x02,0xFF,0x01,0x04,0xFF,0x05,0x00},{0x00,0x01,0xFF,0x02,0x05,0xFF,0x04,0x03}};  	//Motor Code ASVX
	EXTERN const uint8_t hs_to_commutation[2][8] = 	{{0xFF,0x03,0x05,0x04,0x01,0x02,0x00,0xFF},{0xFF,0x00,0x04,0x05,0x02,0x01,0x03,0xFF}};		//Motor Code BLY17
	//EXTERN const uint8_t hs_to_commutation[2][8] = 	{{0xFF,0x05,0x01,0x00,0x03,0x04,0x02,0xFF},{0xFF,0x04,0x02,0x03,0x00,0x05,0x01,0xFF}};		//Motor Code ASPVT
	//EXTERN const uint8_t hs_to_commutation[2][8] = 		{{0xFF,0x04,0x00,0x05,0x02,0x03,0x01,0xFF},{0xFF,0x05,0x03,0x04,0x01,0x00,0x02,0xFF}};		//Motor Code ASRR
	//EXTERN const uint8_t hs_to_commutation[2][8] = 	{{0xFF,0x03,0x05,0x04,0x01,0x02,0x00,0xFF},{0xFF,0x03,0x05,0x04,0x01,0x02,0x00,0xFF}};		//Motor Code ASFG
	//EXTERN const uint8_t hs_to_commutation[2][8] = 		{{0xFF,0x05,0x03,0x04,0x01,0x00,0x02,0xFF},{0xFF,0x05,0x01,0x00,0x03,0x04,0x02,0xFF}};		//Motor Code ASRD
	//EXTERN const uint8_t hs_to_commutation[2][8] = {{0xFF,0x03,0x05,0x04,0x01,0x02,0x00,0xFF},{0xFF,0x01,0x05,0x00,0x03,0x02,0x04,0xFF}};		//Motor Code QDR
	//EXTERN const uint8_t hs_to_commutation[2][8] = {{0xFF,0x03,0x01,0x02,0x05,0x04,0x00,0xFF},{0xFF,0x01,0x03,0x02,0x05,0x00,0x04,0xFF}};		//Motor Code ASPO8
	//EXTERN const uint8_t hs_to_commutation[2][8] = 	{{0xFF,0x00,0x02,0x01,0x04,0x05,0x03,0xFF},{0xFF,0x04,0x02,0x03,0x00,0x05,0x01,0xFF}};		//Motor Code QPO2

#else
	EXTERN const uint8_t slcomp_mux[2][6];
	EXTERN const uint8_t c_pwm_io_state[2][6];
	EXTERN const uint32_t psel_mask[2][6];
	EXTERN const uint32_t psel_mask_pbmux[2][6];
	EXTERN const uint32_t psel_mask_pcmux[2][6];
	EXTERN const uint32_t slcomp_cross_polarity[2][6];
	EXTERN const uint8_t hs_to_commutation[2][8];
#endif

#endif
