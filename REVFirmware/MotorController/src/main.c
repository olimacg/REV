/****************************************************************************
*
*     Main.c file 
*
******************************************************************************/
#include "bldc_common.h"


// Github Test Comment 1
int main(void)
{
      // DO NOT REMOVE: Manually inserts deadtime before execution of all code
      // to allow debugger time to reflash a possibly bricked device
      for(volatile int i = 0; i<1000000; i++);
      
	__disable_irq();
        //Initializations
        
	peripheral_init();
	device_select_init();
	cafe_init();
	ssp_init(SSPC, SSP_MS_SLAVE);
	__enable_irq();

	
        
       
        PAC55XX_GPIOC->MODE.P4 = IO_PUSH_PULL_OUTPUT;
        configure_timer_b_compare_mode();
  
    
	while(1)
		{
                
                
                  
		//while (!millisecond);
		//millisecond = 0;
                
		
                
                
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
