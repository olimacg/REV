//=================================================================================
/// @file     pac5xxx_memory.h
/// @brief    CMSIS Cortex-M Header File for the PAC55XX Memory Controller
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
#ifndef PAC5XXX_MEMORY_H
#define PAC5XXX_MEMORY_H

#define FLASH_LOCK_ALLOW_WRITE_MEMCTL       0xD513B490          // Write this value to FLASHLOCK to allow write to MEMCTL register
#define FLASH_LOCK_ALLOW_WRITE_ERASE_FLASH  0x43DF140A          // Write this value to FLASHLOCK to allow write and erase operations to FLASH
#define FLASH_LOCK_ALLOW_WRITE_SWDFUSE      0x79B4F762          // Write this value to FLASHLOCK to allow write access to INFO2.SWDFUSE to permanently disable SWD
#define FLASH_LOCK_ALLOW_WRITE_SECEN        0x1D855C1E          // Write this value to FLASHLOCK to allow writes to INFO2.SECEN

#define FLASH_START_PAGE_ERASE              0x8C799CA7          // Allow memory controller to start a FLASH page erase operation. 
#define FLASH_ERASE_INFO_3                  0x1266FF45          // Allow erase info-3 flash pages
#define FLASH_START_MASS_PAGE_ERASE         0x09EE76C9          // Start a Mass Erase of all flash memory pages
#define FLASH_START_MASS_PROG_INFO_ERASE    0x856E0E70          // Start a Mass Program and INFO3 Erase
//#define FLASH_SWDFUSE_ACCESS              0x79B4F762          // This define is Deprecated, see FLASH_LOCK_ALLOW_WRITE_SWDFUSE // Write this value to FLASHLOCK to allow writes to SWDFUSE
//#define FLASH_SECEN_ACCESS                0x1D855C1E          // This define is Deprecated, see FLASH_LOCK_ALLOW_WRITE_SECEN   //Write this value to FLASHLOCK to allow writes to SECEN

#define SWDFUSE_ADDRESS                     (PAC55XX_INFO2_FLASH_BASE + 0x3C)
#define SECEN_ADDRESS                       (PAC55XX_INFO2_FLASH_BASE + 0x20)
#define SWD_DISABLE_PERMANENTLY             0xDEAFDEAF                          // Permanent SWD Access Disable

#define FLASH_NUM_PAGES                     128
#define FLASH_PAGE_SIZE_BYTES               1024

// MEMCTL MCLK MUX Select Enumeration Type
typedef enum {
  MEMCTL_MCLK_ROSCCLK   = 0,                  // Select MCLK = ROSCCLK
  MEMCTL_MCLK_HCLKDIV   = 1                   // Select MCLK = HCLKDIV
} MEMCTL_MCLK_TYPE;

//MEMCTL MCLK divider Enumeration Type
typedef enum {
    MEMCTL_MCLK_DIV1 = 0,                   // HCLK /1
    MEMCTL_MCLK_DIV2 = 1,                   // HCLK /2
    MEMCTL_MCLK_DIV3 = 2,                   // HCLK /3
    MEMCTL_MCLK_DIV4 = 3,                   // HCLK /4
    MEMCTL_MCLK_DIV5 = 4,                   // HCLK /5
    MEMCTL_MCLK_DIV6 = 5,                   // HCLK /6
    MEMCTL_MCLK_DIV7 = 6,                   // HCLK /7
    MEMCTL_MCLK_DIV8 = 7,                   // HCLK /8
    MEMCTL_MCLK_DIV9 = 8,                   // HCLK /9
    MEMCTL_MCLK_DIV10 = 9,                  // HCLK /10
    MEMCTL_MCLK_DIV11 = 10,                 // HCLK /11
    MEMCTL_MCLK_DIV12 = 11,                 // HCLK /12
    MEMCTL_MCLK_DIV13 = 12,                 // HCLK /13
    MEMCTL_MCLK_DIV14 = 13,                 // HCLK /14
    MEMCTL_MCLK_DIV15 = 14,                 // HCLK /15
    MEMCTL_MCLK_DIV16 = 15                  // HCLK /16
} MEMCTL_MCLK_DIV_TYPEDEF;

//MEMCTL Wait States Enumeration Type
typedef enum {
    MEMCTL_WSTATE0  = 0,                     // 0  wait states
    MEMCTL_WSTATE1  = 1,                     // 1  wait states
    MEMCTL_WSTATE2  = 2,                     // 2  wait states
    MEMCTL_WSTATE3  = 3,                     // 3  wait states
    MEMCTL_WSTATE4  = 4,                     // 4  wait states
    MEMCTL_WSTATE5  = 5,                     // 5  wait states
    MEMCTL_WSTATE6  = 6,                     // 6  wait states
    MEMCTL_WSTATE7  = 7,                     // 7  wait states
    MEMCTL_WSTATE8  = 8,                     // 8  wait states
    MEMCTL_WSTATE9  = 9,                     // 9  wait states
    MEMCTL_WSTATE10 = 10,                    // 10 wait states
    MEMCTL_WSTATE11 = 11,                    // 11 wait states
    MEMCTL_WSTATE12 = 12,                    // 12 wait states
    MEMCTL_WSTATE13 = 13,                    // 13 wait states
    MEMCTL_WSTATE14 = 14,                    // 14 wait states
    MEMCTL_WSTATE15 = 15                     // 15 wait states
} MEMCTL_FLASH_WSTATE_TYPEDEF;

typedef enum {
    MEMCTL_SEC_LEVEL0  = 0xFF,                 // Security Level 0
    MEMCTL_SEC_LEVEL1  = 0xFE,                 // Security Level 1
    MEMCTL_SEC_LEVEL2  = 0xFC                  // Security Level 2
} MEMCTL_SECURITY_LEVEL_TYPEDEF;

typedef struct
{
    // MEMCTL ( Memory Controller Configuration Register, 400D 0000h)
    union {
        __IO uint32_t w;
        struct {
            __IO uint32_t WSTATE        : 4;     // 3:0        FLASH Read Wait States
            __IO uint32_t MCLKDIV       : 4;     // 7:4        MCLK Divider for FLASH controller
            __IO uint32_t WRITEWORDCNT  : 2;     // 9:8        Write State Machine 32-bit Word Count - Number of words written so far            
                 uint32_t               : 6;     // 15:10       Reserved
            __IO uint32_t SEIE          : 1;     // 16         Single Bit Detection Interrupt Enable (ECC)
            __IO uint32_t DEIE          : 1;     // 17         Dual Bit Detection Interrupt Enable (ECC)
            __IO uint32_t INVADDRIE     : 1;     // 18         Invalid Memory Access Interrupt Enable            
            __IO uint32_t STDBY         : 1;     // 19         FLASH standby Mode
            __IO uint32_t ECCDIS        : 1;     // 20         FLASH ECC Disable
            __IO uint32_t CACHEDIS      : 1;     // 21         FLASH Cache Disable
            __IO uint32_t MCLKSEL       : 1;     // 22
                 uint32_t               : 9;     // 31:23      Reserved
        };
    } MEMCTL;
    
    // MEMSTATUS ( Memory Controller Status Register, 400D 0004h)
    union {
        __IO uint32_t w;
        struct {
            __I  uint32_t WBUSY         : 1;     // 0          Write Busy
            __I  uint32_t EBUSY         : 1;     // 1          Erase Busy
                 uint32_t               : 6;     // 7:2
            __IO uint32_t WRITEWORDCNT  : 2;     // 9:8        Write State Machine 32-bit Word Count - Number of words written so far
                 uint32_t               : 6;     // 15:10
            __IO uint32_t SE            : 1;     // 16         Single-bit Detection Flag
            __IO uint32_t DE            : 1;     // 17         Double-bit Detection Flag
            __IO uint32_t INVADDR       : 1;     // 18         Invalid Address Fetch Flag
                 uint32_t               : 13;    // 31:19
        };
    } MEMSTATUS;
    
    // FLASHLOCK ( FLASH Lock Access Register , 400D 0008h)
    __IO uint32_t FLASHLOCK;
    
    // FLASHPAGE ( FLASH Page Register ,  400D 000Ch)
    union {
        __IO uint32_t w;
        struct {
            __IO uint32_t PAGE          : 7;     // 6:0          FLASH Page Selection
                 uint32_t               : 25;    // 31:7
        };
    } FLASHPAGE;

    // SWDUNLOCK ( SWD Unlock Register , 400D 0010h) 
    __O uint32_t SWDUNLOCK;

    // Reserved 400D 0014h ~400D 001Ch
    uint32_t RESERVED1[3];
    
    // FLASHPERASE ( FLASH Page Erase Register , 400D 0020h)
    __IO uint32_t FLASHERASE;
    
        // Reserved 400D 0024h ~400D 0FCh
    uint32_t RESERVED2[0x37];
    
    __I uint32_t CACHE1ADDR;
    __I uint32_t CACHE1DATA[4];
   
    __I uint32_t CACHE2ADDR;
    __I uint32_t CACHE2DATA[4];

    __I uint32_t CACHE3ADDR;
    __I uint32_t CACHE3DATA[4];

} PAC55XX_MEMCTL_TYPEDEF;

typedef struct
{
    __IO uint32_t UNIQUEID[3];              // 0010 0000h   96 bit Unique ID
} PAC55XX_INFO1_TYPEDEF;

typedef struct
{
    __I uint32_t RESERVED1[2];              // 0010 0400h
    union {                                 // 0010 0408h
        __I uint32_t PACIDR;
        struct {
            __I uint16_t PART_NUM;          // 0010 0408h  Example: value of 0x5523 = PAC5523  (decimal 21795)
            __I uint8_t PART_REV;          // 0010 040Ah
                uint8_t RESERVED2;         // 0010 040Bh  
        };
    };    
    __I  uint32_t RESERVED3[1];             // 0010 040Ch
    __I  uint16_t VMS100;                   // 0010 0410h
    __I  uint16_t VMS200;                   // 0010 0412h
    __I  uint32_t ADCOFFSET;                // 0010 0414h
    __I  uint32_t ADCGAIN;                  // 0010 0418h
    __I  uint16_t TEMPS;                    // 0010 041Ch
    __I  uint16_t FTTEMP;                   // 0010 041Eh
    __IO uint8_t  SECEN;                    // 0010 0420h
         uint8_t  RESERVED4;                // 0010 0421h
         uint16_t RESERVED5;                // 0010 0422h
    __I  uint32_t MAXADDR;                  // 0010 0424h
    __I  uint32_t ROSC;                     // 0010 0428h   Location contains ROSC clock freq * 10
    __I  uint32_t CLKREF;                   // 0010 042Ch
         uint32_t RESERVED6[3];             // 0010 0430h
    __IO uint32_t SWDFUSE;                  // 0010 043Ch
} PAC55XX_INFO2_TYPEDEF;

typedef struct                              // Only used for devices with Security Enabled (SECEN written)
{
    __IO uint32_t RMASK[4];                 // 0010 0800h   Read Mask to make pages unreadable during debug
         uint32_t RESERVED1[4];
    __IO uint32_t WMASK[4];                 // 0010 0820h   Write Mask to make pages unwritable during debug
         uint32_t RESERVED2[4];
    __IO uint8_t  IMASK;                    // 0010 0840h   INFO Mask to make INFO FLASH Pages unreadable
    __IO uint8_t  SECLEVEL;                 // 0010 0841h   Security Level
} PAC55XX_INFO3_TYPEDEF;

#endif  // PAC5XXX_MEMORY_H
