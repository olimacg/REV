//==============================================================================
/// @note
/// Copyright (C) 2017-2019, Qorvo, Inc.
///
/// @par
/// THIS SOFTWARE IS SUBJECT TO A SOURCE CODE LICENSE AGREEMENT WHICH PROVIDES, 
/// AMONG OTHER THINGS:  (i) THAT IT CAN BE USED ONLY TO ADAPT THE LICENSEE'S 
/// APPLICATION TO PAC PROCESSORS SUPPLIED BY QORVO, INC.; 
/// (ii) THAT  IT IS PROVIDED "AS IS" WITHOUT WARRANTY;  (iii) THAT 
/// QORVO, INC. IS NOT LIABLE FOR ANY INDIRECT DAMAGES OR FOR DIRECT
/// DAMAGES EXCEEDING US$1,500;  AND (iv) THAT IT CAN BE DISCLOSED TO AND USED
/// ONLY BY CERTAIN AUTHORIZED PERSONS.
///
//==============================================================================

#ifndef PAC5XXX_SDK_VERSION_H
#define PAC5XXX_SDK_VERSION_H

#define SDK_VERSION_MAJOR           1
#define SDK_VERSION_MINOR           5
#define SDK_VERSION_BUGFIX          1
#define SDK_VERSION_RELEASE_STATE   PAC5XXX_RELEASED  //PAC5XXX_RELEASED, PAC5XXX_RC, PAC5XXX_BETA, PAC5XXX_ALPHA, PAC5XXX_IN_DEVELOPMENT 
#define SDK_VERSION_RELEASE_NUM     0
#define SDK_VERSION_STR             "v1.5.1"

#endif /* PAC5XXX_SDK_VERSION_H */

/*============= Change Log  ===============================================================
===========================
PAC55XX SDK v1.5.1
===========================
- pac5xxx_tile_signal_manager.h - For CAFE_ARCH1, added missing SPECCFG1 and SPECCFG2 registers

===========================
PAC55XX SDK v1.5.0
===========================
- Changed all source file headers to Qorvo, Inc.; Active-Semi was acquired by Qorvo, Inc.
- Added PAC5527 support
- Updated CAN Peripheral header and driver files
- pac5xxx_memory.h: updated PACIDR structure to have correct type for bit fields to eliminate compiler
  warning

===========================
PAC55XX SDK v1.4.0
===========================
Added PAC5524 support

===========================
PAC55XX SDK v1.3.0
===========================
Added PAC5556 support
pac5xxx_tile_power_manager.h - ADDR_FAULTMASK decrecated; use ADDR_FAULTENABLE instead
system_pac5xxx.c - updated to include header files contained in SDK instead of from .pack
pac5xxx_driver_timer.h - added a few more timer CCR set functions     

===========================
PAC55XX SDK v1.2.0
===========================
pac5xxx.h - corrected __NVIC_PRIO_BITS definition to be 3 bits, which gives 8 interrupt priority levels
pac5xxx_adc.h - added back ADCCTL_ADMUX_Type enum and noted that is deprecated and should be replaced by ADC_CHAN_TYPE
pac5xxx_gpio.h - replaced DEBOUNCE with new name CLKSYNC; These bits enable input clock synchronization, it is not a debounce function
device folder added with device headers - include one of these before accessing AFE registers

===========================
PAC55XX SDK v1.1.1
===========================
using __ASM for inline assembly in pac_delay_asm() to be compatible across gcc compilers
updated pac5xxx_driver_timer.c to eliminate compiler warnings
updated pac5xxx_timers.h with QEP interrupt flags as __IO

===========================
PAC55XX SDK v1.1.0
===========================
Improvements
------------
Updated various files with new defines and functions for clearer DTSE/ADC understanding
Added pac_recovery() that allows recovery of a misconfigured device if PE3 is connected to ground

===========================
PAC55XX SDK v1.0.1
===========================
Improvements
------------
- removed Keil startup CRP entry at fixed link location - not present on PAC devices
- fixed GCC RAMFUNC definition to eliminate .data section changed warning
- Corrected data structure entries for ADC DTSE_SEQ_CFG_TYPEDEF and modified pac5xxx_dtse_seq_config_table() function
- Corrected Timer TXCCTL_CCLATCH_CAPTURE_TYPE enum
- Updated INFO Memory Definitions: INFO1, INFO2, INFO3
- Added CCSCTL->LDOEN bit, this bit defaults to 1 at reset and should be 1 when accessing FLASH
- UART: Added UART FIFO Enable function; changed static functions to static inline

===========================
PAC55XX SDK v1.0.0
===========================
- Initial Release

========================================================================================*/