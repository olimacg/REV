/****************************************************************************
 * @file     main.c
 * @brief    Firmware for PAC BLDC controller BEMF sensorless motor
 * @date     13 February 2015
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
#include "bldc_common.h"

/**
 * @brief  Main program that performs PAC configuration and handles real time task management
 *
 * @return none
 *
 */
// DIS IS DA ONE
int main(void)
{
      for(volatile int i = 0; i<1000000; i++);
	__disable_irq();
	peripheral_init();
	device_select_init();
	cafe_init();
	
	__enable_irq();

	
        
        
        ssp_init(SSPC, SSP_MS_SLAVE);
       
        
        
       
       

PAC55XX_GPIOC->MODE.P4 = IO_PUSH_PULL_OUTPUT;
configure_timer_b_compare_mode();
        //int test1; 
        //uint32_t test2;
        
        
    
	while(1)
		{
                
                
                  
		//while (!millisecond);
		millisecond = 0;
                
		
                
                
                set_motor_params(SSPC);
                /*if (motor_
                 if (SPI.DATA_RX.SPEED == 0 && disable motor == 0) {
                    motor_status = motor_active;
                }  */
                /*
		if (motor_status == motor_disabled && SPI.DATA_RX.SPEED)
                      {
                        current_speed = 1;
                        motor_status = motor_active;
                        commutate(firstcomm);
                        
                      }*/
               
       
      
        
	}
}
