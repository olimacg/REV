/****************************************************************************
 *            
 *              Motor Definitions
 * 
 *
 ******************************************************************************/
#ifndef MOTOR_DEF_H
#define MOTOR_DEF_H

//#define	PWM_SWITCH_FREQ_20KHZ 	1
#define	PWM_SWITCH_FREQ_100KHZ 	1

// **********************************************************************************
// Motor Parameters - Can be configured from GUI during Run Time
// **********************************************************************************

#if PWM_SWITCH_FREQ_20KHZ

#define	PWM_SWITCH_FREQ			20

// Speed PI Loop Gains
#define SPEED_PI_TD				FIX16(1024 / 1000)		// 1 ms * 1024 in fix16 format
#define	SPEED_PI_KP				FIX16(1.0)
#define	SPEED_PI_KI				FIX16(5.0)
#define	SPEED_PI_MIN			-500
#define	SPEED_PI_MAX			500

// Current PI Loop Gains
#define IQ_PI_TD				FIX16(1024 / 20000)		// 5 us * 1024 in fix16 format
#define	IQ_PI_KP				FIX16(0.3)
#define	IQ_PI_KI				FIX16(9.0)
#define	IQ_PI_MIN				FIX16(0.1)				//Minimum Duty Cycle to start the motor
#define	IQ_PI_MAX				FIX16(0.95)				//Maximum Duty Cycle

#define	START_BLANKING_CYCLES	15
#define	START_GOOD_SAMPLES		10

#define START_IQ_REF_DEF		40
#define ALIGN_TIME_MS_DEF		10
#define OL_START_HZ_DEF			15
#define	OL_CL_TRANSITION_MS		75						//Number Of Milliseconds after Closed Loop Transition where the START_BLANKING_CYCLES and START_GOOD_SAMPLES are used
#define OL_SWITCHOVER_HZ_DEF	80

#define DEFAULT_OL_ACCEL_PERIOD	5						//Number of ms between speed increase during Open Loop Mode
#define DEFAULT_OL_ACCEL_INC	1						//Number of Hz for the speed increase during Open Loop Mode
#define DEFAULT_CL_ACCEL_PERIOD	1						//Number of ms between speed increase during Closed Loop Mode
#define DEFAULT_CL_ACCEL_INC	5						//Number of Hz for the speed increase during Closed Loop Mode

#define	RUN_BLANKING_CYCLES		10						//Blanking period in TIMER C Cycles
#define	RUN_GOOD_SAMPLES		1						//Number of good samples (in TIMER C Cycles) to acknowledge a BEMF transition

#define	TIMERC_PERIOD			250						//Use to set Timer C Sample Frequency (Freq = TIMERC_FREQ / TIMERC_PERIOD = 50MHz / 150 = 333.333 KHz)
#define	START_SPEED_HZ			300						//Closed Loop Start Speed (ramps from Switchover to this when transitioning from open loop to closed loop)

#define	HALF_DEGREE_ADV_DLY		FIX16(1/120)			// 0.5 degree advance delay factor = 1/2 Comm Adv Delay * 1/2 degree * 1/30 degrees
#define	ADV_DLY_DEF				60

#define	BLDC_DIRECTION			1						//1 for forward rotation, 0 for reverse rotation

#define	SCALE_PERCENT 			FIX16(1/6)				// Sine Wave Commutation Scaling Factor

#endif

#if PWM_SWITCH_FREQ_100KHZ

#define	PWM_SWITCH_FREQ			100
// Speed PI Loop Gains
#define SPEED_PI_TD				FIX16(1024 / 1000)		// 1 ms * 1024 in fix16 format
#define	SPEED_PI_KP				FIX16(0.4)
#define	SPEED_PI_KI				FIX16(1.0)
#define	SPEED_PI_MIN			-250
#define	SPEED_PI_MAX			200

// Current PI Loop Gains
#define	TD_100KHZ				0						//1 for 100 KHz Current PI, 0 for 10 KHz Current PI
#if TD_100KHZ
#define	TD_HZ_DEF				1000					//100 KHz = 10 us DT (1024 / 1000) in FIX16
#else
#define	TD_HZ_DEF				100						//10 KHz = 100 us DT (1024 / 100) in FIX16
#endif
#define IQ_PI_TD				FIX16(1024 / TD_HZ_DEF)	//TD_DEF = 1024*ADC_COUNTER#/(PWMFREQHZ*1000)
#define	IQ_PI_KP				FIX16(0.1)
#define	IQ_PI_KI				FIX16(9.0)
#define	IQ_PI_MIN				FIX16(0.05)				//Minimum Duty Cycle to start the motor
#define	IQ_PI_MAX				FIX16(0.98)				//Maximum Duty Cycle

#define	START_BLANKING_CYCLES	15
#define	START_GOOD_SAMPLES		5

#define	STARTUP_CURRENT			FIX16(0.10)
#define START_IQ_REF_DEF		50
#define ALIGN_TIME_MS_DEF		250
#define OL_START_HZ_DEF			15
#define OL_SWITCHOVER_HZ_DEF	80
#define	OL_CL_TRANSITION_MS		50						//Number Of Milliseconds after Closed Loop Transition where the START_BLANKING_CYCLES and START_GOOD_SAMPLES are used

#define DEFAULT_OL_ACCEL_PERIOD	5						//Number of ms between speed increase during Open Loop Mode
#define DEFAULT_OL_ACCEL_INC	1						//Number of Hz for the speed increase during Open Loop Mode
#define DEFAULT_CL_ACCEL_PERIOD	1						//Number of ms between speed increase during Closed Loop Mode
#define DEFAULT_CL_ACCEL_INC	3						//Number of Hz for the speed increase during Closed Loop Mode

#define	RUN_BLANKING_CYCLES		2						//Blanking period in TIMER C Cycles
#define	RUN_GOOD_SAMPLES		1						//Number of good samples (in TIMER C Cycles) to acknowledge a BEMF transition

#define	TIMERC_PERIOD			400						//Use to set Timer C Sample Frequency (Freq = TIMERC_FREQ / TIMERC_PERIOD = 100MHz / 500 = 200 KHz)
#define	START_SPEED_HZ			150						//Closed Loop Start Speed (ramps from Switchover to this when transitioning from open loop to closed loop)

#define	HALF_DEGREE_ADV_DLY		FIX16(1/120)			// 0.5 degree advance delay factor = 1/2 Comm Adv Delay * 1/2 degree * 1/30 degrees
#define	ADV_DLY_DEF				60

#define	BLDC_DIRECTION			1						//1 for forward rotation, 0 for reverse rotation

#define	SCALE_PERCENT 			FIX16(0.17)				// Sine Wave Commutation Scaling Factor

#endif

#endif
