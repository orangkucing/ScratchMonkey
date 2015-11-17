// -*- mode: c++; tab-width: 4; indent-tabs-mode: nil -*-
//
// ScratchMonkey 0.1        - STK500v2 compatible programmer for Arduino
//
// File: SMoXPROG.cpp       - XPROG Interface
//

// This part of ScratchMonkey is written by Hisashi Ito <info at mewpro.cc> (c) 2015
// in order to support HVprog2, an STK500 clone open hardware that you can buy or make.
// http://www.mewpro.cc
//
// Derived from source codes of LUFA AVRISP mkII clone, avrdude, USBasp
// and ATtiny4_5_9_10_20_40Programmer.ino

// The same license as main part applies.

#ifndef _SMO_XPROG_
#define _SMO_XPROG_

namespace SMoXPROG {
    void XPROG();
    void XPROG_SetMode();
    extern struct param_t {
        uint32_t NVMBase;
        uint16_t EEPageSize;
        uint8_t NVMCMD; // Non-Volatile Memory Command Register
        uint8_t NVMCSR; // Non-Volatile Memory Control and Status Register
        uint16_t unknown; // since AVRStudio 5.1
    } XPRGParam;
} // namespace SMoXPROG

#include "SMoCommand.h"
#define XPRG_Body (SMoCommand::gBody+1)

// XPROG modes
#define XPRG_MODE_PDI                       0
#define XPRG_MODE_JTAG                      1
#define XPRG_MODE_TPI                       2

// XPROG commands
#define XPRG_CMD_ENTER_PROGMODE             0x01
#define XPRG_CMD_LEAVE_PROGMODE             0x02
#define XPRG_CMD_ERASE                      0x03
#define XPRG_CMD_WRITE_MEM                  0x04
#define XPRG_CMD_READ_MEM                   0x05
#define XPRG_CMD_CRC                        0x06
#define XPRG_CMD_SET_PARAM                  0x07

// Memory types
#define XPRG_MEM_TYPE_APPL                   1
#define XPRG_MEM_TYPE_BOOT                   2
#define XPRG_MEM_TYPE_EEPROM                 3
#define XPRG_MEM_TYPE_FUSE                   4
#define XPRG_MEM_TYPE_LOCKBITS               5
#define XPRG_MEM_TYPE_USERSIG                6
#define XPRG_MEM_TYPE_FACTORY_CALIBRATION    7

// Erase types
#define XPRG_ERASE_CHIP                      1
#define XPRG_ERASE_APP                       2
#define XPRG_ERASE_BOOT                      3
#define XPRG_ERASE_EEPROM                    4
#define XPRG_ERASE_APP_PAGE                  5
#define XPRG_ERASE_BOOT_PAGE                 6
#define XPRG_ERASE_EEPROM_PAGE               7
#define XPRG_ERASE_USERSIG                   8
#define XPRG_ERASE_CONFIG                    9  // TPI only, prepare fuse write

// Write mode flags
#define XPRG_MEM_WRITE_ERASE                 0
#define XPRG_MEM_WRITE_WRITE                 1

// CRC types
#define XPRG_CRC_APP                         1
#define XPRG_CRC_BOOT                        2
#define XPRG_CRC_FLASH                       3

// Error codes
#define XPRG_ERR_OK                          0
#define XPRG_ERR_FAILED                      1
#define XPRG_ERR_COLLISION                   2
#define XPRG_ERR_TIMEOUT                     3

// XPROG parameters of different sizes
// 4-byte address
#define XPRG_PARAM_NVMBASE                  0x01
// 2-byte page size
#define XPRG_PARAM_EEPPAGESIZE              0x02
// 1-byte NVMCMD register
#define XPRG_PARAM_NVMCMD_REG               0x03 // was XPRG_PARAM_TPI_3
// 1-byte NVMCSR register
#define XPRG_PARAM_NVMCSR_REG               0x04 // was XPRG_PARAM_TPI_4
// 2-byte, undocumented
#define XPRG_PARAM_UNKNOWN_1                0x05

#endif /* _SMO_XPROG_ */
