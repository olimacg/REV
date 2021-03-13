/****************************************************************************
 * @file     sl_functions.h
 * @brief    Sensorless App Specific Functions
 * @date     22 September 2015
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
#define INCLUDE_EXTERNS
#include "bldc_common.h"

#ifdef HALL_SENSOR_APP

/**
 * @brief	This function disables all high and low-side PWMs by driving them low.
 * @return none
 *
 */
void motor_pwm_disable(void)
{
	__disable_irq();

	//NVIC_DisableIRQ(TimerD_IRQn);
	//NVIC_DisableIRQ(TimerC_IRQn);

	PAC55XX_TIMERD->CTL.BASEIE = 0;
	PAC55XX_TIMERC->CTL.BASEIE = 0;


	PAC55XX_GPIOB->OUT.w = 0;			//PORTB OUT GPIO = 0;
	PAC55XX_SCC->PBMUXSEL.w = 0;		//PORTB PSEL is ALL GPIO's

	
	app_status &= ~(status_motor_enabled + status_closed_loop);
	SMS_State = SMS_Idle;
	open_loop = 1;
	ADCSM_State = ADCSM_Idle;
	pwm_duty = 1;

	__enable_irq();
}

/**
 * @brief	This function generates schedules a commutation sequence when a HS transition is detected
 *
 * @return none
 *
 */
APP_RAMFUNC void aa_commutate(uint32_t ccrn)
{

	hall_sensor_value = PAC55XX_GPIOD->IN.w & 0x07;

	next_commutation_state = hs_to_commutation[motordir][hall_sensor_value];

	switch (ccrn)
		{
		case pwmc0:
			motorspeed = PAC55XX_TIMERC->CCTR0.w;
		break;

		case pwmc1:
			motorspeed = PAC55XX_TIMERC->CCTR1.w;
		break;

		case pwmc2:
			motorspeed = PAC55XX_TIMERC->CCTR2.w;
		break;

		case pwmc4:
			motorspeed = PAC55XX_TIMERC->CCTR4.w;
		break;

		case pwmc5:
			motorspeed = PAC55XX_TIMERC->CCTR5.w;
		break;

		case pwmc6:
			motorspeed = PAC55XX_TIMERC->CCTR6.w;
		break;

		case pwmc_base:
			next_commutation_state++;
			if (next_commutation_state > 5)
				{
				next_commutation_state = 0;
				}

			motorspeed = 0x7FFF;
		break;

		case firstcomm:
			break;
		}

   	//Speed Computation Code Segment
   	//Speed is computed in a per sector basis, measuring the amount of time between Hall Sensor edges.
   	//Real Motor Speed is motorspeed multiplied by 6
   	//If Not Angle Advance, motor speed per sector can be computed after commutation
   	//If Angle Advance Option was compiled in, then the motor speed needs to be computed before scheduling the commutation point

	avg_speed_index++;
	if (avg_speed_index > 5)
		avg_speed_index = 0;
	avg_speed_array[avg_speed_index] = motorspeed;
	//Average Speed Computation Code Segment
	//On each sector, the time between Hall Sensor transitions is measured. Each one of these values is stored in a six word deep array.
	//The Average Speed, avg_speed, is the average of the last six measured sectors.
	//avg_speed is used to regulate speed with the Speed PI Loop

	int i;
	avg_speed = 0;
	for (i=0;i<=5;i++)
		{
		avg_speed += avg_speed_array[i];
		}
	avg_speed = fix16_mul_new_16_16(avg_speed, ONE_SIXTH_F16);

	// Use to calculate commutation advanced delay
	// GUI sends a number from 0 to 60 which corresponds to 0 to 30 degrees in 0.5 degrees increment (GUI# / 2)
	// Commutation Advance Delay = motorspeed * half_degree_adv_delay_factor * GUI#
	// half_degree_adv_delay_factor = 0.00833333 = 1/2 * 1/60

	commutation_time = (fix16_mul_new_16_16(avg_speed << 16, commutation_advanced_rise) >> 16) & 0x7FFF;

	__disable_irq();
	PAC55XX_TIMERC->CTL.MODE = TxCTL_MODE_DISABLED;
	PAC55XX_TIMERD->CTL.MODE = TxCTL_MODE_DISABLED;

	PAC55XX_TIMERC->CTL.CLR = 1;
	PAC55XX_TIMERD->CTL.CLR = 1;

	PAC55XX_TIMERD->PRD.w = commutation_time;			//Initialize Timer D Period

	PAC55XX_TIMERD->INT.BASEIF = 1;
	PAC55XX_TIMERD->CTL.BASEIE = 1;

	PAC55XX_TIMERC->CTL.CLR = 0;
	PAC55XX_TIMERD->CTL.CLR = 0;

	PAC55XX_TIMERC->CTL.MODE = TxCTL_MODE_UP;
	PAC55XX_TIMERD->CTL.MODE = TxCTL_MODE_UP;
	__enable_irq();

}

/**
 * @brief	This function generates a commutation sequence when a HS transition is detected
 *
 * @return none
 *
 */
APP_RAMFUNC void commutate(uint32_t ccrn)
{


	hall_sensor_value = PAC55XX_GPIOD->IN.w & 0x07;


        uint8_t hs_to_commutation[2][8] = 	{{0xFF,0x03,0x05,0x04,0x01,0x02,0x00,0xFF},{0xFF,0x00,0x04,0x05,0x02,0x01,0x03,0xFF}};
        
        //motordir = 1;
          
	next_commutation_state = hs_to_commutation[motor_dir][hall_sensor_value];

	if (next_commutation_state != 0xFF)
		{


		PAC55XX_SCC->PBMUXSEL.w = psel_mask[motor_dir][next_commutation_state];	        // Set peripheral select state
		PAC55XX_GPIOB->OUT.w = c_pwm_io_state[motor_dir][next_commutation_state];
		}

	__disable_irq();
   	PAC55XX_TIMERC->CTL.CLR = 1;
   	PAC55XX_TIMERC->CTL.CLR = 0;
   	PAC55XX_TIMERC->CTL.MODE = TxCTL_MODE_UP;
   	__enable_irq();

	switch (ccrn)
		{
		case pwmc0:
			motorspeed = PAC55XX_TIMERC->CCTR0.w;
		break;

		case pwmc1:
			motorspeed = PAC55XX_TIMERC->CCTR1.w;
		break;

		case pwmc2:
			motorspeed = PAC55XX_TIMERC->CCTR2.w;
		break;

		case pwmc4:
			motorspeed = PAC55XX_TIMERC->CCTR4.w;
		break;

		case pwmc5:
			motorspeed = PAC55XX_TIMERC->CCTR5.w;
		break;

		case pwmc6:
			motorspeed = PAC55XX_TIMERC->CCTR6.w;
		break;

		case pwmc_base:
			motorspeed = 0x7FFF;
		break;

		case firstcomm:
			break;
		}

	avg_speed_index++;
	if (avg_speed_index > 5)
		avg_speed_index = 0;
	avg_speed_array[avg_speed_index] = motorspeed;

	int i;
	avg_speed = 0;
	for (i=0;i<=5;i++)
		{
		avg_speed += avg_speed_array[i];
		}
	avg_speed = fix16_mul_new_16_16(avg_speed, ONE_SIXTH_F16);
}


void accelerate(void){
  
  if (motor_speed_scaler >  5 + current_speed)
    {
        current_speed += ACCEL_FACTOR;
    }
  else if (motor_speed_scaler <  current_speed - 5) 
    {
      current_speed -= ACCEL_FACTOR;
    }
  
        PAC55XX_TIMER_SEL->CCTR4.CTR = current_speed;
        PAC55XX_TIMER_SEL->CCTR5.CTR = current_speed;
        PAC55XX_TIMER_SEL->CCTR6.CTR = current_speed; //OG test 2      
        
}

#endif
