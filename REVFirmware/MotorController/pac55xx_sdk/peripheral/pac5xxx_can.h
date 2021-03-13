//=================================================================================
/// @file     pac5xxx_can.h
/// @brief    CMSIS Cortex-M Header File for the PAC55XX CAN
///
/// @note
/// Copyright (C) 2017-2019, Qorvo, Inc.
///
/// THIS SOFTWARE IS SUBJECT TO A SOURCE CODE LICENSE AGREEMENT WHICH PROVIDES,
/// AMONG OTHER THINGS:  (i) THAT IT CAN BE USED ONLY TO ADAPT THE LICENSEE'S
/// APPLICATION TO PAC PROCESSORS SUPPLIED BY QORVO, INC.;
/// (ii) THAT  IT IS PROVIDED "AS IS" WITHOUT WARRANTY;  (iii) THAT
/// QORVO, INC. IS NOT LIABLE FOR ANY INDIRECT DAMAGES OR FOR DIRECT
/// DAMAGES EXCEEDING US$1,500;  AND (iv) THAT IT CAN BE DISCLOSED TO AND USED
/// ONLY BY CERTAIN AUTHORIZED PERSONS.
///
//=================================================================================

#ifndef PAC5XXX_CAN_H
#define PAC5XXX_CAN_H

// CAN Arbitration Lost Capture Enumeration Type
typedef enum {

  ARBITRATION_LOST_IN_ID28_P_10  = 0,
  ARBITRATION_LOST_IN_ID28_P_9   = 1,
  ARBITRATION_LOST_IN_ID28_P_8   = 2,
  ARBITRATION_LOST_IN_ID28_P_7   = 3, 
  ARBITRATION_LOST_IN_ID28_P_6   = 4,
  ARBITRATION_LOST_IN_ID28_P_5   = 5,
  ARBITRATION_LOST_IN_ID28_P_4   = 6,
  ARBITRATION_LOST_IN_ID28_P_3   = 7, 
  ARBITRATION_LOST_IN_ID28_P_2   = 8,
  ARBITRATION_LOST_IN_ID28_P_1   = 9,
  ARBITRATION_LOST_IN_ID28_P_0   = 10, 
  ARBITRATION_LOST_IN_SRTR_P_RTR = 11,   
  ARBITRATION_LOST_IN_IDE_BIT    = 12,   
  ARBITRATION_LOST_IN_ID17       = 13,   
  ARBITRATION_LOST_IN_ID16       = 14,   
  ARBITRATION_LOST_IN_ID15       = 15,   
  ARBITRATION_LOST_IN_ID14       = 16,   
  ARBITRATION_LOST_IN_ID13       = 17,   
  ARBITRATION_LOST_IN_ID12       = 18,   
  ARBITRATION_LOST_IN_ID11       = 19,   
  ARBITRATION_LOST_IN_ID10       = 20,   
  ARBITRATION_LOST_IN_ID9        = 21,   
  ARBITRATION_LOST_IN_ID8        = 22,   
  ARBITRATION_LOST_IN_ID7        = 23,   
  ARBITRATION_LOST_IN_ID6        = 24,   
  ARBITRATION_LOST_IN_ID5        = 25,   
  ARBITRATION_LOST_IN_ID4        = 26,   
  ARBITRATION_LOST_IN_ID3        = 27,   
  ARBITRATION_LOST_IN_ID2        = 28,   
  ARBITRATION_LOST_IN_ID1        = 29,   
  ARBITRATION_LOST_IN_ID0        = 30,   
  ARBITRATION_LOST_IN_RTR        = 31   
} CAN_ALC_TYPE;



// ISR_SR_CMR_MR Register bit defines
#define MR_AFM		(1 << 0)     // Hardware acceptance filter scheme
#define MR_LOM		(1 << 1)     // Listen only mode
#define MR_RM		(1 << 2)     // Reset mode

#define CMR_AT		(1 << 9)     // Abort transmission
#define CMR_TR		(1 << 10)    // Transmit request

#define SR_BS		(1 << 16)    // Bus Off Status
#define SR_ES		(1 << 17)    // Error Status
#define SR_TS		(1 << 18)    // Transmit Status
#define SR_RS		(1 << 19)    // Receive Status
#define SR_TBS		(1 << 21)    // Transmit Buffer Status
#define SR_RBS		(1 << 23)    // Receive Buffer Status
#define SR_DSO		(1 << 22)    // Data Overrun Status

#define ISR_DOI		(1 << 24)   // Data Overflow Interrupt
#define ISR_BEI		(1 << 25)   // Bus Error Interrupt
#define ISR_TI		(1 << 26)   // Transmit Interrupt
#define ISR_RI		(1 << 27)   // Receive Interrupt
#define ISR_EPI		(1 << 28)   // Error Passive Interrupt
#define ISR_EWI		(1 << 29)   // Error Warning Interrupt
#define ISR_ALI		(1 << 30)   // Arbitration Lost Interrupt

// PAC55XX CAN Typedef
typedef volatile struct
{
    // (CAN ISR_SR_CMR_MR Register, 400A 0000h):  MR, CMR, SR, and ISR register definitions
    union 
    {
        __IO uint32_t ISR_SR_CMR_MR;        // Use to write the entire 32-bit MR, CMR, SR, ISR register

        // MR (CAN Mode Register, 400A 0000h): 
        union                                   
        {
            struct {
                __IO uint32_t b     : 8;    // 7:0      Byte Access               
                     uint32_t       : 24;   // 31:8     Reserved
            };
            struct {
                __IO uint32_t AFM   : 1;    // 0        Hardware acceptance filter scheme
                __IO uint32_t LOM   : 1;    // 1        Listen only mode
                __IO uint32_t RM    : 1;    // 2        Reset mode
                     uint32_t       : 5;    // 7:3      Reserved                  
                     uint32_t       : 24;   // 31:8     Reserved
            };
        } MR;

        // CMR (CAN Command Register, 400A 0001h): 
        union                                   
        {
            struct {
                     uint32_t       : 8;    // 7:0      Reserved
                __IO uint32_t b     : 8;    // 15:8     Byte Access           
                     uint32_t       : 16;   // 31:16    Reserved
            };
            struct {
                     uint32_t       : 8;    // 7:0      Reserved
                     uint32_t       : 1;    // 8        Reserved
                __IO uint32_t AT    : 1;    // 9        Abort transmission
                __IO uint32_t TR    : 1;    // 10       Transmit request
                     uint32_t       : 5;    // 15:11    Reserved
                     uint32_t       : 16;   // 31:16    Reserved
            };
        } CMR;

        // SR (CAN Status Register, 400A 0002h): 
        union                                   
        {
            struct {
                     uint32_t       : 16;   // 15:0     Reserved
                __I  uint32_t b     : 8;    // 23:16    Byte Access
                     uint32_t       : 8;    // 31:24    Reserved
            };
            struct {
                     uint32_t       : 16;   // 15:0     Reserved
                __I  uint32_t BS    : 1;    // 16       Bus Off Status
                __I  uint32_t ES    : 1;    // 17       Error Status
                __I  uint32_t TS    : 1;    // 18       Transmit Status
                __I  uint32_t RS    : 1;    // 19       Receive Status
                     uint32_t       : 1;    // 20       Reserved
                __I  uint32_t TBS   : 1;    // 21       Transmit Buffer Status
                __I  uint32_t DSO   : 1;    // 22       Data Overrun Status
                __I  uint32_t RBS   : 1;    // 23       Receive Buffer Status
                     uint32_t       : 8;    // 31:24    Reserved
            };
        } SR;

        // ISR (CAN Interrupt Status/Ack Register, 400A 0003h):
        union                                   
        {
            struct {
                     uint32_t       : 24;   // 23:0     Reserved
                __IO uint32_t b     : 8;    // 31:24    Byte Access        
            };
            struct {
                     uint32_t       : 24;   // 23:0     Reserved
                __IO uint32_t DOI   : 1;    // 24       Data Overflow Interrupt
                __IO uint32_t BEI   : 1;    // 25       Bus Error Interrupt
                __IO uint32_t TI    : 1;    // 26       Transmit Interrupt
                __IO uint32_t RI    : 1;    // 27       Receive Interrupt
                __IO uint32_t EPI   : 1;    // 28       Error Passive Interrupt
                __IO uint32_t EWI   : 1;    // 29       Error Warning Interrupt
                __IO uint32_t ALI   : 1;    // 30       Arbitration Lost Interrupt
                     uint32_t       : 1;    // 31       Reserved
            };
        } ISR;
    };

    // (CAN BTR1_BTR0_RMC_IMR Register, 400A 0004h): IMR, RMC, BTR0, BTR1 register definitions
    union
    {
    	__IO uint32_t BTR1_BTR0_RMC_IMR;

    	// IMR (CAN CAN Interrupt Mask Register, 400A 0004h):
        union
        {
            struct {
                __IO uint32_t b     : 8;    // 7:0      Byte Access
                     uint32_t       : 24;   // 31:8     Reserved
            };
            struct {
                __IO uint32_t DOIM  : 1;    // 0        DOI Interrupt Mask
                __IO uint32_t BEIM  : 1;    // 1        BEI Interrupt Mask
                __IO uint32_t TIM   : 1;    // 2        TI Interrupt Mask
                __IO uint32_t RIM   : 1;    // 3        RI Interrupt Mask
                __IO uint32_t EPIM  : 1;    // 4        EPI Interrupt Mask
                __IO uint32_t EWIM  : 1;    // 5        EWI Interrupt Mask
                __IO uint32_t ALIM  : 1;    // 6        ALI Interrupt Mask
                     uint32_t       : 1;    // 7        Reserved
                     uint32_t       : 24;   // 31:8     Reserved
            };
        } IMR;

    	// RMC (CAN Receive Message Counter Register, 400A 0005h):
        union
        {
            struct {
            	     uint32_t       : 8;    // 7:0      Reserved
                __I  uint32_t b     : 8;    // 15:8     Byte Access
                     uint32_t       : 16;   // 31:16    Reserved
            };
            struct {
                     uint32_t       : 8;    // 7:0      Reserved
                __I  uint32_t RMC   : 5;    // 12:8     Number of frames stored in the receive FIFO
                     uint32_t       : 3;    // 15:13    Reserved
                     uint32_t       : 16;   // 31:16    Reserved
            };
        } RMC;

    	// BTR0 (CAN Bus Timing 0 Register, 400A 0006h):
        union
        {
            struct {
      	             uint32_t       : 16;   // 15:0     Reserved
                __IO uint32_t b     : 8;    // 23:16    Byte Access
                     uint32_t       : 8;    // 31:24    Reserved
            };
            struct {
            	     uint32_t       : 16;   // 15:0     Reserved
                __IO uint32_t BRP   : 6;    // 21:16    Baud Rate Pre-scaler
                __IO uint32_t SJW   : 2;    // 23:22    Synchronization jump width
                     uint32_t       : 8;    // 31:24    Reserved
            };
        } BTR0;

    	// BTR1 (CAN Bus Timing 1 Register, 400A 0007h):
        union
        {
            struct {
 	                 uint32_t       : 24;   // 23:0     Reserved
                __IO uint32_t b     : 8;    // 31:24    Byte Access
            };
            struct {
            	     uint32_t       : 24;   // 23:0     Reserved
                __IO uint32_t TSEG1 : 4;    // 27:24    Number of clock cycles per Time Segment 1
                __IO uint32_t TSEG2 : 3;    // 30:28    Number of clock cycles per Time Segment 2
                __IO uint32_t SAM   : 1;    // 31       Number of bus level samples
            };
        } BTR1;
    };

    // CANTXBUF (CAN Transmit Buffer Register, 400A 0008h):
    __IO uint32_t TXBUF;

    // CANRXBUF (CAN Receive Buffer Register, 400A 000Ch):
    __I  uint32_t RXBUF;

    // CANACR (CAN Acceptance Code Register, 400A 0010h):
    __IO uint32_t ACR;

    // CANACRM (CAN Acceptance Mask Register, 400A 0014h):
    __IO uint32_t AMR;
 
    // ALC_TXERR_RXERR_ECC (CAN Error Code Capture Register, 400A 0018h): ECC, RXERR, TXERR, ALC register definitions
    union
    {
    	__IO uint32_t ALC_TXERR_RXERR_ECC;

    	// ECC (CAN Error Code Capture Register, 400A 0018h):
        union
        {
            struct {
                __I  uint32_t b     : 8;    // 7:0      Byte Access
                     uint32_t       : 24;   // 31:8     Reserved
            };
            struct {
                __I  uint32_t BER   : 1;    // 0        Bit error occurred
                __I  uint32_t STFER : 1;    // 1        Stuff error occurred
                __I  uint32_t CRCER : 1;    // 2        CRC error occurred
                __I  uint32_t FRMER : 1;    // 3        Form error occurred
                __I  uint32_t ACKER : 1;    // 4        ACK error occurred
                __I  uint32_t EDIR  : 1;    // 5        Direction of transfer while error occurred
                __I  uint32_t TXWRN : 1;    // 6        Set when CANTXERR >= 96
                __I  uint32_t RXWRN : 1;    // 7        Set when CANRXERR >= 96
            	     uint32_t       : 24;   // 31:8     Reserved
            };
        } ECC;

    	// RXERR (CAN Receive Error Counter Register, 400A 0019h):
        union
        {
            struct {
      	             uint32_t       : 8;    // 7:0      Reserved
                __I  uint32_t b     : 8;    // 15:8     Byte Access
                     uint32_t       : 16;   // 31:16    Reserved
            };
            struct {
            	     uint32_t          : 8; // 7:0      Reserved
                __I  uint32_t CANRXERR : 8; // 15:8     Receive error counter
                     uint32_t          : 16;// 31:16    Reserved
            };
        } RXERR;

    	// TXERR (CAN Transmit Error Counter Register, 400A 001Ah):
        union
        {
            struct {
      	             uint32_t       : 16;   // 15:0     Reserved
                __I  uint32_t b     : 8;    // 23:16    Byte Access
                     uint32_t       : 8;    // 31:24    Reserved
            };
            struct {
            	     uint32_t          : 16;// 15:0     Reserved
                __I  uint32_t CANTXERR : 8; // 23:16    Transmit error counter
                     uint32_t          : 8; // 31:24    Reserved
            };
        } TXERR;

    	// ALC (CAN Arbitration Lost Code Capture Register, 400A 001Bh):
        union
        {
            struct {
      	             uint32_t       : 24;   // 23:0     Reserved
                __I  uint32_t b     : 8;    // 31:24    Byte Access
            };
            struct {
            	     uint32_t       : 24;   // 23:0     Reserved
                __I  uint32_t CANALC: 5;    // 28:24    Arbitration Lost Bit Number
                     uint32_t       : 3;    // 31:29    Reserved
            };
        } ALC;
    };
} PAC55XX_CAN_TYPEDEF;

#endif  // PAC5XXX_CAN_H
