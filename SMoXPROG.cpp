// -*- mode: c++; tab-width: 4; indent-tabs-mode: nil -*-
//
// ScratchMonkey 0.1        - STK500v2 compatible programmer for Arduino
//
// File: SMoXPROG.cpp       - XPROG for TPI and PDI 
//

// This part of ScratchMonkey is written by Hisashi Ito <info at mewpro.cc> (c) 2015
// in order to support HVprog2, an STK500 clone open hardware that you can buy or make.
// http://www.mewpro.cc
//
// Derived from source codes of LUFA AVRISP mkII clone, avrdude, USBasp
// and ATtiny4_5_9_10_20_40Programmer.ino

// The same license as main part applies.

#include "SMoCommand.h"
#include "SMoGeneral.h"
#include "SMoXPROG.h"
#include "SMoTPI.h"
#include "SMoPDI.h"

#define XPRG_Body (SMoCommand::gBody+1)

struct SMoXPROG::param_t SMoXPROG::XPRGParam;

static uint8_t SelectedProtocol = XPRG_MODE_TPI;


static uint16_t
SetParam()
{
    uint8_t param = XPRG_Body[1];
    
    switch (param) {
    case XPRG_PARAM_NVMBASE: // PDI only
        SMoXPROG::XPRGParam.NVMBase = XPRG_Body[2]<<24 | XPRG_Body[3]<<16 | XPRG_Body[4]<<8 | XPRG_Body[5];
        break;
    case XPRG_PARAM_EEPPAGESIZE: // PDI only
        SMoXPROG::XPRGParam.EEPageSize = XPRG_Body[2]<<8 | XPRG_Body[3];
        break;
    case XPRG_PARAM_NVMCMD_REG: // TPI only
        SMoXPROG::XPRGParam.NVMCMD = XPRG_Body[2];
        break;
    case XPRG_PARAM_NVMCSR_REG: // TPI only
        SMoXPROG::XPRGParam.NVMCSR = XPRG_Body[2];
        break;
    case XPRG_PARAM_FLASHPAGESIZE: // PDI only (Atmel Studio 5.1 or later)
        SMoXPROG::XPRGParam.FlashPageSize = XPRG_Body[2]<<8 | XPRG_Body[3];
        break;
    default:
        // not implemented
        break;
    }
    XPRG_Body[1] = XPRG_ERR_OK;
    return 2;
}

void
SMoXPROG::XPROG()
{
    uint8_t XPROGCommand = XPRG_Body[0];

    uint16_t answerlen;
    switch (SelectedProtocol) {
    case XPRG_MODE_TPI:
        switch (XPROGCommand) {
        case XPRG_CMD_ENTER_PROGMODE:
            answerlen = SMoTPI::EnterProgmode();
            break;
        case XPRG_CMD_LEAVE_PROGMODE:
            answerlen = SMoTPI::LeaveProgmode();
            break;
        case XPRG_CMD_ERASE:
            answerlen = SMoTPI::Erase();
            break;
        case XPRG_CMD_WRITE_MEM:
            answerlen = SMoTPI::WriteMem();
            break;
        case XPRG_CMD_READ_MEM:
            answerlen = SMoTPI::ReadMem();
            break;
        case XPRG_CMD_CRC:
            answerlen = SMoTPI::CRC();
            break;
        case XPRG_CMD_SET_PARAM:
            answerlen = SetParam();
            break;
        default:
            SMoCommand::SendResponse(STATUS_CMD_FAILED);
            return;
        }
        break;
    case XPRG_MODE_PDI:
        switch (XPROGCommand) {
        case XPRG_CMD_ENTER_PROGMODE:
            SMoGeneral::gVoltage = 33; // report 3.3V from now on if queried
            SMoXPROG::XPRGParam.FlashPageSize = 0; // assume the old method
            answerlen = SMoPDI::EnterProgmode();
            break;
        case XPRG_CMD_LEAVE_PROGMODE:
            SMoGeneral::gVoltage = 50; // report 5.0V from now on if queried
            answerlen = SMoPDI::LeaveProgmode();
            break;
        case XPRG_CMD_ERASE:
            answerlen = SMoPDI::Erase();
            break;
        case XPRG_CMD_WRITE_MEM:
            answerlen = SMoPDI::WriteMem();
            break;
        case XPRG_CMD_READ_MEM:
            answerlen = SMoPDI::ReadMem();
            break;
        case XPRG_CMD_CRC:
            answerlen = SMoPDI::CRC();
            break;
        case XPRG_CMD_SET_PARAM:
            answerlen = SetParam();
            break;
        default:
            SMoCommand::SendResponse(STATUS_CMD_FAILED);
            return;
        }
        break;
    default:
        SMoCommand::SendResponse(STATUS_CMD_FAILED);
        return;
    }
    SMoCommand::SendResponse(XPROGCommand, answerlen+1);  
}

void
SMoXPROG::XPROG_SetMode()
{
    int8_t mode = XPRG_Body[0];
 
    SelectedProtocol = mode;
    SMoCommand::SendResponse();
}
