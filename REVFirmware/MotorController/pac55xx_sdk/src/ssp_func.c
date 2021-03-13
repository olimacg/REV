//=============================================================================
//
//                        SSP (SPI) FUNCTIONS - C
//
//=============================================================================
#define INCLUDE_EXTERNS
#include "bldc_common.h"


//==============================================================================
///@brief
///     SSPC_SCLK <--> PE0
///     SSPC_SS   <--> PE1
///     SSPC_MOSI <--> PE2
///     SSPC_MISO <--> PE3
///
//==============================================================================


void SSPC_IO_Select_PE0123(SSP_MS_TYPE ms_mode)
{
    if (ms_mode == SSP_MS_MASTER)
    {
        PAC55XX_GPIOE->MODE.P0 = IO_PUSH_PULL_OUTPUT;           // SCLK
        PAC55XX_GPIOE->MODE.P1 = IO_PUSH_PULL_OUTPUT;           // SS
        PAC55XX_GPIOE->MODE.P2 = IO_PUSH_PULL_OUTPUT;           // MOSI
        PAC55XX_GPIOE->MODE.P3 = IO_HIGH_IMPEDENCE_INPUT;       // MISO

        PAC55XX_SCC->PEPUEN.P0 = 0;
        PAC55XX_SCC->PEPUEN.P1 = 0;
        PAC55XX_SCC->PEPUEN.P2 = 0;
        PAC55XX_SCC->PEPUEN.P3 = 1;
    }
    else
    {
        PAC55XX_GPIOE->MODE.P0 = IO_HIGH_IMPEDENCE_INPUT;       // SCLK
        PAC55XX_GPIOE->MODE.P1 = IO_HIGH_IMPEDENCE_INPUT;       // SS
        PAC55XX_GPIOE->MODE.P2 = IO_HIGH_IMPEDENCE_INPUT;       // MOSI
        PAC55XX_GPIOE->MODE.P3 = IO_PUSH_PULL_OUTPUT;           // MISO

        PAC55XX_SCC->PEPUEN.P0 = 1;
        PAC55XX_SCC->PEPUEN.P1 = 1;
        PAC55XX_SCC->PEPUEN.P2 = 1;
        PAC55XX_SCC->PEPUEN.P3 = 0;
    }

    PAC55XX_SCC->PEMUXSEL.w &= 0xFFFF0000;                      // Clear Port Pin selection
    PAC55XX_SCC->PEMUXSEL.w |= 0x00005555;                      // Set Port Pin as SSP
}


//==============================================================================
///@brief
///     Choose the SSP and Enable the IO you want
///
///@param[in]
///     SSP Type:
///          SSPA
///          SSPB
///          SSPC
///          SSPD
///     SSP_MS_TYPE:
///          MASTER
///          SLAVE
///
//==============================================================================
void ssp_io_config(SSP_TYPE ssp, SSP_MS_TYPE ms_mode)
{
    switch (ssp)
    {
        case SSPA:
            // Select ssp A peripheral choose one!
            //SSPA_IO_Select_PA3456(ms_mode);
            break;

        case SSPB:
            // Select ssp B peripheral choose one!
            //SSPB_IO_Select_PA3456(ms_mode);
//            SSPB_IO_Select_PC0123(ms_mode);
//            SSPB_IO_Select_PC4567(ms_mode);
//            SSPB_IO_Select_PE4567(ms_mode);
//            SSPB_IO_Select_PF0123(ms_mode);
            break;

        case SSPC:
            // Select ssp C peripheral choose one!
//            SSPC_IO_Select_PC0123(ms_mode);
//            SSPC_IO_Select_PC4567(ms_mode);
//            SSPC_IO_Select_PD0123(ms_mode);
            SSPC_IO_Select_PE0123(ms_mode);
            break;

        case SSPD:
            // Select ssp D peripheral choose one!
            //SSPD_IO_Select_PD4567(ms_mode);
//            SSPD_IO_Select_PE4567(ms_mode);
//            SSPD_IO_Select_PF4567(ms_mode);
//            SSPD_IO_Select_PG0123(ms_mode);
//            SSPD_IO_Select_PG4567(ms_mode);
            break;

        default:
            break;
    }
}

//==============================================================================
///@brief
///     enable the interrupt
///
///@param[in]
///     SSP Type:
///         SSPA
///         SSPB
///         SSPC
///         SSPD
///
//==============================================================================
void ssp_interrupt_enable(SSP_TYPE ssp)
{
    switch (ssp)
    {
        case SSPA:
            NVIC_ClearPendingIRQ(USARTA_IRQn);
            NVIC_SetPriority(USARTA_IRQn, 3);
            NVIC_EnableIRQ(USARTA_IRQn);
            break;

        case SSPB:
            NVIC_ClearPendingIRQ(USARTB_IRQn);
            NVIC_SetPriority(USARTB_IRQn, 3);
            NVIC_EnableIRQ(USARTB_IRQn);
            break;

        case SSPC:
            NVIC_ClearPendingIRQ(USARTC_IRQn);
            NVIC_SetPriority(USARTC_IRQn, 3);
            NVIC_EnableIRQ(USARTC_IRQn);
            break;

        case SSPD:
            NVIC_ClearPendingIRQ(USARTD_IRQn);
            NVIC_SetPriority(USARTD_IRQn, 3);
            NVIC_EnableIRQ(USARTD_IRQn);
            break;

        default:
            break;
    }
}

void ssp_init(SSP_TYPE ssp, SSP_MS_TYPE ms_mode)
{
    PAC55XX_SSP_TYPEDEF *ssp_ptr;

    switch (ssp)
    {
        case SSPA:
            ssp_ptr = PAC55XX_SSPA;
            PAC55XX_SCC->CCSCTL.USAMODE = USART_MODE_SSP;       // SSP mode
            break;

        case SSPB:
            ssp_ptr = PAC55XX_SSPB;
            PAC55XX_SCC->CCSCTL.USBMODE = USART_MODE_SSP;       // SSP mode
            break;

        case SSPC:
            ssp_ptr = PAC55XX_SSPC;
            PAC55XX_SCC->CCSCTL.USCMODE = USART_MODE_SSP;       // SSP mode
            break;

        case SSPD:
            ssp_ptr = PAC55XX_SSPD;
            PAC55XX_SCC->CCSCTL.USDMODE = USART_MODE_SSP;       // SSP mode
            break;

        default:
            ssp_ptr = PAC55XX_SSPD;
            break;
    }

    // SSPCLK = PCLK / ((SSPxCLK.M + 1)*SSPxCLK.N) = 150000000 / ((2+1) x 100) = 500kHz
    ssp_ptr->CLK.M = 2;               
    ssp_ptr->CLK.N = 2;                    // N nust be event value from 2 to 254
    ssp_ptr->CON.FRF = SSP_FRAME_FORMAT_SPI;                 // Frame Format, SPI frame format
    ssp_ptr->CON.MS = ms_mode;                               // Master/Slave mode, master mode
    ssp_ptr->CON.LSBFIRST = SSP_ENDIAN_MSB;                  // Endian Order, MSB transmit 1st
    ssp_ptr->CON.LBM = SSP_LP_NORMAL;                        // Loobback Mode, no loopback mode
    ssp_ptr->CON.CPH = SSP_PUT_PHASE_2;                  // Clock Out Phase, SPI captuers data sat 2nd edge transition of the frame
    ssp_ptr->CON.CPO = SSP_CLK_POLARITY_HIGH;                // Clock Out Polarity, SPI clock active high
    ssp_ptr->CON.DSS = SSP_DATA_SIZE_8;                   // Data Size Select, 16 bit data
    ssp_ptr->CON.SOD = SSP_OUTPUT_DRIVE;                 // Slave Output Enable
    //ssp_ptr->SSCR.SELSS = 0;

    ssp_io_config(ssp, ms_mode);
    ssp_interrupt_enable(ssp);

    // Set the interrupt for Reading data
//    ssp_ptr->IMSC.TXIM = 1;                                  // Enable TX FIFO interrupt
//    ssp_ptr->ICLR.RORIC = 1;                                 // Clear RX Overrun interrupt
 //   ssp_ptr->ICLR.RTIC = 1;                                  // Clear RX Timeout interrupt
//    ssp_ptr->IMSC.RXIM = 1;                                  // Enable RX FIFO interrupt
//    ssp_ptr->IMSC.RORIM = 1;                                 // Enable RX Overrun interrupt
//   ssp_ptr->IMSC.RTIM = 1;                                  // Enable RX Timeout interrupt

    ssp_ptr->CON.SSPEN = SSP_CONTROL_ENABLE;                 // SSP Enable
}

//==============================================================================
///@brief
///     Write a 16-bit data to SSP manually
///
///@param[in]
///     SSP_TYPE:
///         SSPA
///         SSPB
///         SSPC
///         SSPD
///     data: The data to write.
///
///@param[out]
///     result: the result of the SSP write
///
///@retval
///     0: All is OK.
///     others: Some error occurs.
///
//==============================================================================
uint32_t ssp_write_one(SSP_TYPE ssp, uint16_t data)
{
    uint32_t result = 0;
    //uint32_t result = PAC5XXX_OK;
    uint32_t wait_tick = 0;
    PAC55XX_SSP_TYPEDEF *ssp_ptr;

    switch (ssp)
    {
        case SSPA:
            ssp_ptr = PAC55XX_SSPA;
            break;

        case SSPB:
            ssp_ptr = PAC55XX_SSPB;
            break;

        case SSPC:
            ssp_ptr = PAC55XX_SSPC;
            break;

        case SSPD:
            ssp_ptr = PAC55XX_SSPD;
            break;

        default:
            ssp_ptr = PAC55XX_SSPD;
            break;
    }

    // Write 16-bits Data
    ssp_ptr->DAT.DATA = data;

    while (ssp_ptr->STAT.TNF == 0)
    {
        wait_tick++;
        if (wait_tick > DF_SSP_BUSY_TICK)
        {
            //result = PAC5XXX_ERROR;
            result = 1;
            break;
        }
    }

    return result;
}

//==============================================================================
///@brief
///     Write a serial 16-bit data to SSP manually
///
///@param[in]
///     SSP_TYPE:
///         SSPA
///         SSPB
///         SSPC
///         SSPD
///     data: The data point to write.
///     byte_num: the number to write.
///
///@param[out]
///     result: the result of the SSP byte write
///
///@retval
///     0: All is OK.
///     others: Some error occurs.
///
//==============================================================================
uint32_t ssp_write_multi(SSP_TYPE ssp, uint16_t *data, uint32_t byte_num)
{       
    uint32_t result = 0;
    //uint32_t result = PAC5XXX_OK;
    uint32_t wait_tick = 0;
    uint32_t byte_num_temp;
    PAC55XX_SSP_TYPEDEF *ssp_ptr;

    switch (ssp)
    {
        case SSPA:
            ssp_ptr = PAC55XX_SSPA;
            break;

        case SSPB:
            ssp_ptr = PAC55XX_SSPB;
            break;

        case SSPC:
            ssp_ptr = PAC55XX_SSPC;
            break;

        case SSPD:
            ssp_ptr = PAC55XX_SSPD;
            break;

        default:
            ssp_ptr = PAC55XX_SSPD;
            break;
    }

    for (byte_num_temp=0; byte_num_temp<byte_num; byte_num_temp++)
    {
        // Write 16-bits data
        ssp_ptr->DAT.DATA = *data++;

        while (ssp_ptr->STAT.TNF == 0)
        {
            wait_tick++;
            if (wait_tick > DF_SSP_BUSY_TICK)
            {
                //result = PAC5XXX_ERROR;
                result = 1;
                break;
            }
        }

        wait_tick = 0;
    }

    return result;
}

uint32_t ssp_read_one(SSP_TYPE ssp)
{

  PAC55XX_SSP_TYPEDEF *ssp_ptr;
  
  switch (ssp)
    {
        case SSPA:
            ssp_ptr = PAC55XX_SSPA;
            break;

        case SSPB:
            ssp_ptr = PAC55XX_SSPB;
            break;

        case SSPC:
            ssp_ptr = PAC55XX_SSPC;
            break;

        case SSPD:
            ssp_ptr = PAC55XX_SSPD;
            break;

        default:
            ssp_ptr = PAC55XX_SSPD;
            break;
    }
  
    //uint32_t result = ssp_ptr->DAT.DATA;
      
    return ssp_ptr->DAT.DATA;
    
  
}

uint32_t ssp_available(SSP_TYPE ssp)
{
  PAC55XX_SSP_TYPEDEF *ssp_ptr;
  
  switch (ssp)
    {
        case SSPA:
            ssp_ptr = PAC55XX_SSPA;
            break;

        case SSPB:
            ssp_ptr = PAC55XX_SSPB;
            break;

        case SSPC:
            ssp_ptr = PAC55XX_SSPC;
            break;

        case SSPD:
            ssp_ptr = PAC55XX_SSPD;
            break;

        default:
            ssp_ptr = PAC55XX_SSPD;
            break;
    }
    
  uint32_t result = 0;
  if (ssp_ptr->STAT.RNE) { result = 1; }

   return result;
  
}


void set_motor_params(SSP_TYPE ssp)
{
  if (ssp_available(ssp) ) 
  {
    SPI.DATA_RX.w = ssp_read_one(ssp);

        // Speed Set
        
        //uint32_t motor_pwm_duty = (uint32_t)( PAC55XX_TIMER_SEL->PRD.w * SPI.DATA_RX.SPEED / 257 );
        
        
        // Direction Set
        
        if( !SPI.DATA_RX.SPEED )
        {
        motor_dir = SPI.DATA_RX.DIRECTION;
        }
        
        // Braking Set 
        
        braking = SPI.DATA_RX.BRAKING;

        // Disable Set
        disable_motor = SPI.DATA_RX.DISABLE;
      
     
    
    
    if ( disable_motor == 0 )   // disable_motor == 0 && && motor_status == motor_active)
    {
        
        
        motor_speed_scaler = (( PAC55XX_TIMER_SEL->PRD.w >> 5) * (uint32_t)((SPI.DATA_RX.SPEED + 1)));
        
        //current_speed = motor_speed_scaler;
        accelerate();
        
        //PAC55XX_TIMER_SEL->CCTR4.CTR = current_speed;
        //PAC55XX_TIMER_SEL->CCTR5.CTR = current_speed;
        //PAC55XX_TIMER_SEL->CCTR6.CTR = current_speed;
       
        
       
     
        
        // testing
        ssp_write_one(SSPC, SPI.DATA_RX.w );
        //ssp_write_one(SSPC, avg_speed);

        //led pwm
        PAC55XX_TIMERB->CCTR4.CTR = (uint32_t)( PAC55XX_TIMERB->PRD.w * (current_speed + 1 )/1000);
       
    }/* else {
      
        //oc_reset();

        motor_pwm_disable();
        current_speed = 1;
        motor_status = motor_disabled;
        
        PAC55XX_TIMER_SEL->CCTR4.CTR = current_speed;
        PAC55XX_TIMER_SEL->CCTR5.CTR = current_speed;
        PAC55XX_TIMER_SEL->CCTR6.CTR = current_speed; //OG test 2       
      
    }*/
      
     
  }
    
  
}
