// -*- mode: c++; tab-width: 4; indent-tabs-mode: nil -*-
//
// ScratchMonkey 0.1        - STK500v2 compatible programmer for Arduino
//
// File: SMoHVSP.cpp        - High Voltage Serial Programming
//                            (for MCUs with fewer than 20 pins)
//
// Copyright (c) 2013 Matthias Neeracher <microtherion@gmail.com>
// All rights reserved.
//
// See license at bottom of this file or at
// http://opensource.org/licenses/bsd-license.php
//

// Modified by Hisashi Ito <info at mewpro.cc> (c) 2015
// in order to support HVprog2, an STK500 clone open hardware that you can buy or make.
// http://www.mewpro.cc

#include "SMoHVSP.h"
#include "SMoCommand.h"
#include "SMoGeneral.h"
#include "SMoConfig.h"

#ifdef DEBUG_HVSP
#include "SMoDebug.h"
#endif

#include <Arduino.h>

enum {
    HVSP_VCC   = SMO_SVCC,
    HVSP_RESET = SMO_HVRESET,
#if SMO_LAYOUT==SMO_LAYOUT_HVPROG2
    HVSP_SDI   =  0,
    HVSP_SII   =  1,
    HVSP_SDO   =  2,
    HVSP_SCI   = 15,
#define HVSP_TOGGLE_SCI    do { PORTD &= ~_BV(7); PORTD |= _BV(7); } while (0)
#else
    HVSP_SDI   =  8,
    HVSP_SII   =  9,
    HVSP_SDO   = 12,
    HVSP_SCI   = 13,
#if SMO_LAYOUT==SMO_LAYOUT_STANDARD
#define HVSP_TOGGLE_SCI    do { PORTB |= _BV(5); PORTB &= ~_BV(5); } while (0)
#elif SMO_LAYOUT==SMO_LAYOUT_LEONARDO
#define HVSP_TOGGLE_SCI    do { PORTC |= _BV(7); PORTC &= ~_BV(7); } while (0)
#elif SMO_LAYOUT==SMO_LAYOUT_MEGA
#define HVSP_TOGGLE_SCI    do { PORTB |= _BV(7); PORTB &= ~_BV(7); } while (0)
#endif
#endif
};

// Control patterns, expressed as index values for the control stack
//
enum {
    sLoadCommand    =  0, // 0x4C sLoadCommand + sLowByte
    sLoadAddr       =  1, // 0x0C sLoadAddr + sLowByte
    //                 2, // 0x1C sLoadAddr + sHighByte
    sLoadData       =  3, // 0x2C sLoadData + sLowByte
    //                 4, // 0x3C sLoadData + sHighByte
    sCommitData     =  5, // 0x64 sCommitData + sLowByte
    //                 6, // 0x74 sCommitData + sHighByte
    //                 7, // 0x66 sCommitData + sExtByte (0x00 in older attiny's)
    sEnableRead     =  8, // 0x68 sEnableRead + sLowByte
    //                 9, // 0x78 sEnableRead + sHighByte
    //                       0x68 n/a
    sFuseRead       = 11, // 0x68 sFuseRead + sLowByte
    //                12, // 0x7A sFuseRead + sHighByte  (0x00 in older attiny's)
    //                13, // 0x6A sFuseRead + sExtByte   (0x00 in older attiny's)
    sSignatureRead  = 14, // 0x68 sSignatureRead + sLowByte
    //                15, // 0x78 sSignatureRead + sHighByte (OSCCAL)
    //                16, // 0x78 sFuseRead + sExt2Byte (LOCK)
    //                17, // 0x7D sPageLoad - sHighByte  (0x00 in older attiny's)
    sPageLoad       = 18, // 0x6D sPageLoad - sLowByte
    sDone           = 19, // 0x0C bitmask

    sChipErase      = 20, // 0x80 Chip Erase command byte
    sWriteFuseBits  = 21, // 0x40 Write Fuse Bits command byte
    sWriteLockBits  = 22, // 0x20 Write Lock Bits command byte
    sWriteFlash     = 23, // 0x10 Write Flash command byte
    sWriteEEPROM    = 24, // 0x11 Write EEPROM command byte
    sReadSignature  = 25, // 0x08 Read Signature Bytes and Calibration Byte command byte
    sReadFuseLock   = 26, // 0x04 Read Fuse and Lock Bits command byte
    sReadFlash      = 27, // 0x02 Read Flash command byte
    sReadEEPROM     = 28, // 0x03 Read EEPROM command byte
    //                       0x08 n/a
    //                       0x04 n/a

    sInit           = 31, // 0x0F if the device has 14 pins

    sLowByte        = 0,
    sHighByte       = 1,
    sExtByte        = 2,
    sExt2Byte       = 5,
#define HVSPControlPattern(c, b)   (SMoGeneral::gControlStack[(c) + (b)])
#define HVSPControlDone(c, b)      (SMoGeneral::gControlStack[(c) + (b)] | SMoGeneral::gControlStack[sDone])
};

inline bool
HVSPBit(bool instrInBit, bool dataInBit)
{
    digitalWrite(HVSP_SII, instrInBit);
    digitalWrite(HVSP_SDI, dataInBit);
    HVSP_TOGGLE_SCI;
    uint8_t dataOutBit = digitalRead(HVSP_SDO);

    return dataOutBit;
}

static uint8_t
HVSPTransfer(uint8_t instrIn, uint8_t dataIn=0)
{
#ifdef DEBUG_HVSP
    SMoDebug.print("Byte ");
    SMoDebug.print(instrIn, HEX);
    SMoDebug.print(" ");
    SMoDebug.print(dataIn, HEX);
#endif
    //
    // First bit, data out only
    //
    uint8_t dataOut = HVSPBit(0, 0);

    //
    // Next bits, data in/out
    //
    for (char i=0; i<7; ++i) {
        dataOut = dataOut << 1 | HVSPBit((instrIn & 0x80) != 0, (dataIn & 0x80) != 0);
        instrIn <<= 1;
        dataIn  <<= 1;
    }
    //
    // Last data out bit
    //
    HVSPBit(instrIn & 0x80, dataIn & 0x80);
    
    //
    // Two stop bits
    //
    HVSPBit(0, 0);
    HVSPBit(0, 0);

#ifdef DEBUG_HVSP
    SMoDebug.print(" -> ");
    SMoDebug.println(dataOut, HEX);
#endif
    return dataOut;
}

static bool
HVSPPollWait(uint8_t pollTimeout)
{
    uint32_t time = millis();
    while (!digitalRead(HVSP_SDO))
        if (millis() - time > (uint32_t)pollTimeout) 
            return false;
    return true;
}

void
SMoHVSP::EnterProgmode()
{
#ifdef DEBUG_HVSP
    SMoDebugInit();
#endif

    const uint8_t   stabDelay   = SMoCommand::gBody[1];
    const uint8_t   progModeDelay = SMoCommand::gBody[2];
    const uint8_t   synchCycles  = SMoCommand::gBody[3];
    // const uint8_t   latchCycles = SMoCommand::gBody[4];
    const uint8_t   toggleVtg   = SMoCommand::gBody[5];
    const uint8_t   powerOffDelay = SMoCommand::gBody[6];
    const uint8_t   resetDelayMs = SMoCommand::gBody[7];
    const uint8_t   resetDelayUs = SMoCommand::gBody[8];

    delay(progModeDelay);
    // power off target
    digitalWrite(HVSP_VCC, LOW);    
    pinMode(HVSP_VCC, OUTPUT);
    delayMicroseconds(250);
    // target RESET = 0V
    digitalWrite(HVSP_RESET, HIGH);
    pinMode(HVSP_RESET, OUTPUT);
    // set control pins
#if SMO_LAYOUT==SMO_LAYOUT_HVPROG2
#ifdef TCCR2
    TCCR2  = 0;
#else
    TCCR2B = 0;
    TCCR2A = 0;
#endif
#else
    TCCR1B = 0;
    TCCR1A = 0;
#endif
    pinMode(HVSP_SCI, INPUT);
    HVSP_TOGGLE_SCI;
    pinMode(HVSP_SCI, OUTPUT);
#if SMO_LAYOUT==SMO_LAYOUT_HVPROG2
    digitalWrite(SMO_HVENABLE, HIGH);
    pinMode(SMO_HVENABLE, OUTPUT); // enable 12V
    // for 14pin AVRs
    DDRC = SMoGeneral::gControlStack[sInit];
    PORTC = 0;
#endif
    digitalWrite(HVSP_SDI, LOW);
    pinMode(HVSP_SDI, OUTPUT);
    digitalWrite(HVSP_SII, LOW);
    pinMode(HVSP_SII, OUTPUT);
    digitalWrite(HVSP_SDO, LOW); // Prog_enable[2]
    pinMode(HVSP_SDO, OUTPUT);
    // make sure HVSP_VCC is 0V
    delay(powerOffDelay);
    if (toggleVtg) {
#if defined(SMO_AVCC)
        uint32_t time = millis();
        while (analogRead(SMO_AVCC) > 50) {   // wait until HVSP_VCC become lower than 0.3V
            if (millis() - time > DEFAULTTIMEOUT) // timeout
                break;
        }
#else
        delay(100);
#endif
    }
    // power on target
    digitalWrite(HVSP_VCC, HIGH);
    delay(resetDelayMs);
    delayMicroseconds(resetDelayUs * 10 + 250); // add extra 250 microseconds
    // toggle SCI
    for (uint8_t i=0; i<synchCycles; ++i) {
        HVSP_TOGGLE_SCI;
    }
    // apply 12V to HVSP_RESET
    digitalWrite(HVSP_RESET, LOW);
    delay(stabDelay);
    pinMode(HVSP_SDO, INPUT);
    
    SMoCommand::SendResponse();
}

void
SMoHVSP::LeaveProgmode()
{
    //const uint8_t   stabDelay  = SMoCommand::gBody[1];
    const uint8_t   resetDelay = SMoCommand::gBody[2];

    digitalWrite(HVSP_VCC, LOW);
    digitalWrite(HVSP_RESET, HIGH);

    delay(resetDelay);
    
#if SMO_LAYOUT==SMO_LAYOUT_HVPROG2
    digitalWrite(SMO_HVENABLE, LOW); // disable 12V
    digitalWrite(HVSP_VCC, HIGH);
    digitalWrite(HVSP_RESET, LOW);
#endif
    //delay(stabDelay);
    SMoCommand::SendResponse();
}

void
SMoHVSP::ChipErase()
{
    const uint8_t pollTimeout   = SMoCommand::gBody[1];
    const uint8_t eraseTime     = SMoCommand::gBody[2];

    HVSPTransfer(HVSPControlPattern(sLoadCommand, sLowByte), SMoGeneral::gControlStack[sChipErase]);
    HVSPTransfer(HVSPControlPattern(sCommitData, sLowByte));
    HVSPTransfer(HVSPControlDone(sCommitData, sLowByte));

    if (pollTimeout) {
        if (!HVSPPollWait(pollTimeout)) {
            SMoCommand::SendResponse(STATUS_RDY_BSY_TOUT);
            return;
        }
    } else {
        delay(eraseTime);
    }
    SMoCommand::SendResponse();
}

static void
ProgramMemory(bool flash)
{
    uint16_t        numBytes    = SMoCommand::gBody[1] << 8 | SMoCommand::gBody[2];
    const uint8_t   mode        = SMoCommand::gBody[3];
    const uint8_t   pollTimeout = SMoCommand::gBody[4];
    const uint8_t * data        = &SMoCommand::gBody[5];

    uint16_t pageMask = (1 << ((mode & 0x0E ? mode & 0x0E : 0x10) >> 1) - (flash ? 1 : 0)) - 1;
    int8_t b;
    HVSPTransfer(HVSPControlPattern(sLoadCommand, sLowByte), flash ? SMoGeneral::gControlStack[sWriteFlash] : SMoGeneral::gControlStack[sWriteEEPROM]);
    HVSPTransfer(HVSPControlPattern(sLoadAddr, sHighByte), SMoGeneral::gAddress.c[1]);
    HVSPTransfer(HVSPControlPattern(sLoadAddr, sLowByte), SMoGeneral::gAddress.c[0]);
    do { 
        b = (!(numBytes & 1) || !flash) ? sLowByte : sHighByte;
        HVSPTransfer(HVSPControlPattern(sLoadData, b), *data);
        if (!(mode & 1)) { // Byte mode
            HVSPTransfer(HVSPControlPattern(sCommitData, b));
            HVSPTransfer(HVSPControlDone(sCommitData, b));
            if (!HVSPPollWait(pollTimeout))
                goto TIMEOUT_ProgramMemory;
        } else { // assert PAGEL
            HVSPTransfer(HVSPControlPattern(sPageLoad, -b));
            HVSPTransfer(HVSPControlDone(sCommitData, b));
        }
        data++;
        if (!(--numBytes & 1) || !flash) {
            SMoGeneral::gAddress.d.addr++;
            if (mode & 1) { // Page mode
                if (!(SMoGeneral::gAddress.d.addr & pageMask) && mode & 0x80 || (numBytes == 0 && mode & 0x40)) { // Write page to memory
                    HVSPTransfer(HVSPControlPattern(sCommitData, sLowByte));
                    HVSPTransfer(HVSPControlDone(sCommitData, sLowByte));
                    if (!HVSPPollWait(pollTimeout))
                        goto TIMEOUT_ProgramMemory;
                }
            }
            if (!numBytes)
                break;
            if (SMoGeneral::gAddress.c[0] == 0)
                HVSPTransfer(HVSPControlPattern(sLoadAddr, sHighByte), SMoGeneral::gAddress.c[1]);
            HVSPTransfer(HVSPControlPattern(sLoadAddr, sLowByte), SMoGeneral::gAddress.c[0]);
        }
    } while (numBytes);
    if (mode & 0x40)
        HVSPTransfer(HVSPControlPattern(sLoadCommand, sLowByte), 0x00);
    SMoCommand::SendResponse();
    return;
TIMEOUT_ProgramMemory:
    SMoCommand::SendResponse(STATUS_RDY_BSY_TOUT);
}

static void
ReadMemory(bool flash)
{
    uint16_t    numBytes    = SMoCommand::gBody[1] << 8 | SMoCommand::gBody[2];
    uint8_t *   dataOut     = &SMoCommand::gBody[2];
    
    int8_t b;
    HVSPTransfer(HVSPControlPattern(sLoadCommand, sLowByte), flash ? SMoGeneral::gControlStack[sReadFlash] : SMoGeneral::gControlStack[sReadEEPROM]);
    HVSPTransfer(HVSPControlPattern(sLoadAddr, sHighByte), SMoGeneral::gAddress.c[1]);
    HVSPTransfer(HVSPControlPattern(sLoadAddr, sLowByte), SMoGeneral::gAddress.c[0]);
    do {
        b = (!(numBytes & 1) || !flash) ? sLowByte : sHighByte;
        HVSPTransfer(HVSPControlPattern(sEnableRead, b));
        *dataOut++ = HVSPTransfer(HVSPControlDone(sEnableRead, b));
        if (!(--numBytes & 1) || !flash) {
            SMoGeneral::gAddress.d.addr++;
            if (!numBytes)
                break;
            if (SMoGeneral::gAddress.c[0] == 0)
                HVSPTransfer(HVSPControlPattern(sLoadAddr, sHighByte), SMoGeneral::gAddress.c[1]);
            HVSPTransfer(HVSPControlPattern(sLoadAddr, sLowByte), SMoGeneral::gAddress.c[0]);
        }
    } while (numBytes);
    *dataOut++ = STATUS_CMD_OK;
    SMoCommand::SendResponse(STATUS_CMD_OK, dataOut - &SMoCommand::gBody[0]);
}

void
SMoHVSP::ProgramFlash()
{
    ProgramMemory(true);
}

void
SMoHVSP::ReadFlash()
{
    ReadMemory(true);
}

void
SMoHVSP::ProgramEEPROM()
{
    ProgramMemory(false);
}

void
SMoHVSP::ReadEEPROM()
{
    ReadMemory(false);
}

static void 
ProgramFuseLock(uint8_t command, uint8_t byteSel)
{
    const uint8_t   value       = SMoCommand::gBody[2];
    const uint8_t   pollTimeout = SMoCommand::gBody[3];
    
    HVSPTransfer(HVSPControlPattern(sLoadCommand, sLowByte), command);
    HVSPTransfer(HVSPControlPattern(sLoadData, sLowByte), value);
    HVSPTransfer(HVSPControlPattern(sCommitData, byteSel));
    HVSPTransfer(HVSPControlDone(sCommitData, byteSel));
    // ATtiny11 doesn't generate any activity on SDO pin.
    // So just ignore the return value even if timeout occurs.
    HVSPPollWait(pollTimeout);
    SMoCommand::SendResponse(STATUS_CMD_OK);
}

static void
ReadFuseLock(uint8_t byteSel)
{
    uint8_t * dataOut   = &SMoCommand::gBody[2];

    HVSPTransfer(HVSPControlPattern(sLoadCommand, sLowByte), SMoGeneral::gControlStack[sReadFuseLock]);
    HVSPTransfer(HVSPControlPattern(sFuseRead, byteSel));   
    *dataOut = HVSPTransfer(HVSPControlDone(sFuseRead, byteSel));

    SMoCommand::SendResponse(STATUS_CMD_OK, 3);
}

void
SMoHVSP::ProgramFuse()
{
    const uint8_t addr = SMoCommand::gBody[1];
    
    ProgramFuseLock(SMoGeneral::gControlStack[sWriteFuseBits], addr);
}

void
SMoHVSP::ReadFuse()
{
    const uint8_t addr = SMoCommand::gBody[1];
    
    ReadFuseLock(addr);
}

void
SMoHVSP::ProgramLock()   
{
    ProgramFuseLock(SMoGeneral::gControlStack[sWriteLockBits], sLowByte);
}

void
SMoHVSP::ReadLock()       
{
    ReadFuseLock(sExt2Byte);
}

static void
ReadSignatureCal(uint8_t addr, uint8_t byteSel)
{
    uint8_t * dataOut   = &SMoCommand::gBody[2];

    HVSPTransfer(HVSPControlPattern(sLoadCommand, sLowByte), SMoGeneral::gControlStack[sReadSignature]);
    HVSPTransfer(HVSPControlPattern(sLoadAddr, sLowByte), addr);
    HVSPTransfer(HVSPControlPattern(sSignatureRead, byteSel));
    *dataOut = HVSPTransfer(HVSPControlDone(sSignatureRead, byteSel));

    SMoCommand::SendResponse(STATUS_CMD_OK, 3);
}

void
SMoHVSP::ReadSignature()  
{
    const uint8_t addr = SMoCommand::gBody[1];
    
    ReadSignatureCal(addr, sLowByte);
}

void
SMoHVSP::ReadOscCal()     
{
    ReadSignatureCal(0x00, sHighByte);
}

//
// LICENSE
//
// Redistribution and use in source and binary forms, with or without modification, 
// are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice, this 
//    list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright notice, 
//    this list of conditions and the following disclaimer in the documentation 
//    and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
