// -*- mode: c++; tab-width: 4; indent-tabs-mode: nil -*-
//
// ScratchMonkey 0.1        - STK500v2 compatible programmer for Arduino
//
// File: SMoISP.cpp         - In-System Programming commands
//
// Copyright (c) 2013 Matthias Neeracher <microtherion@gmail.com>
// All rights reserved.
//
// See license at bottom of this file or at
// http://opensource.org/licenses/bsd-license.php
//
// Derived from Randall Bohn's ArduinoISP sketch
//

// Modified by Hisashi Ito <info at mewpro.cc> (c) 2015
// in order to support HVprog2, an STK500 clone open hardware that you can buy or make.
// http://mewpro.cc/2016/01/20/how-to-use-hvprog2/

#include <SPI.h>

#include "SMoISP.h"
#include "SMoGeneral.h"
#include "SMoCommand.h"
#include "SMoConfig.h"
#ifdef DEBUG_ISP
#include "SMoDebug.h"
#endif

//
// Pin definitions
//
enum {
#if SMO_LAYOUT==SMO_LAYOUT_STANDARD
    ISP_RESET       = SS,
    MCU_CLOCK       = 9,    // OC1A    
#elif SMO_LAYOUT==SMO_LAYOUT_LEONARDO
    ISP_RESET       = 10,   // no dedicated SS pin on Arduino Micro
    MCU_CLOCK       = 9,    // OC1A
#elif SMO_LAYOUT==SMO_LAYOUT_MEGA
    ISP_RESET       = SS,
    MCU_CLOCK       = 11,   // OC1A
#elif SMO_LAYOUT==SMO_LAYOUT_HVPROG2
    ISP_RESET       = SMO_HVRESET,
    MCU_CLOCK       = 15,   // OC2A or OC2
#endif
};

#if SMO_LAYOUT==SMO_LAYOUT_HVPROG2
#define ISPAssertReset() \
    do { \
        digitalWrite(ISP_RESET, !SMoGeneral::gResetPolarity); \
        digitalWrite(ISP_RESET, SMoGeneral::gResetPolarity); \
    } while (0);
#define ISPDeassertReset()   digitalWrite(ISP_RESET, !SMoGeneral::gResetPolarity);
#else
#define ISPAssertReset() \
    do { \
        digitalWrite(ISP_RESET, SMoGeneral::gResetPolarity); \
        digitalWrite(ISP_RESET, !SMoGeneral::gResetPolarity); \
    } while (0);
#define ISPDeassertReset()   digitalWrite(ISP_RESET, SMoGeneral::gResetPolarity);
#endif

const SPISettings ISPSPISetting[3] = {
    SPISettings(2000000, MSBFIRST, SPI_MODE0),
    SPISettings(500000, MSBFIRST, SPI_MODE0),
    SPISettings(125000, MSBFIRST, SPI_MODE0),
};
    
//                                                                                         
// If an MCU has been set to use the 125kHz internal oscillator, 
// regular SPI speeds are much too fast, so we do a software 
// emulation that's deliberately slow.
//
static int sSPILimpMode    = 0;
const int kMaxLimp         = 8; // Slowest we'll try is 256µs, ~1kHz bit clock

static uint8_t 
SPITransfer(uint8_t out)
{
    if (!sSPILimpMode)
        return SPI.transfer(out); // Hardware SPI
        
    const int kQuarterCycle = 1 << sSPILimpMode; 
    uint8_t in = 0;
    for (int i=0; i<8; ++i) {
        digitalWrite(MOSI, (out & 0x80) != 0);
        out <<= 1;
        delayMicroseconds(kQuarterCycle);
        digitalWrite(SCK, HIGH);
        delayMicroseconds(kQuarterCycle);
        in = in << 1 | digitalRead(MISO);
        delayMicroseconds(kQuarterCycle);
        digitalWrite(SCK, LOW);
        delayMicroseconds(kQuarterCycle);        
    }
    return in;
}

static uint8_t 
SPITransaction(const uint8_t * sendData, int8_t responseIndex = 3)
{
    uint8_t response;

#ifdef DEBUG_ISP
    SMoDebug.print("SPI ");
#endif
    for (int8_t ix=0; ix<4; ++ix) {
#ifdef DEBUG_ISP
        SMoDebug.print(*sendData, HEX);
#endif
        uint8_t recv = SPITransfer(*sendData++);
#ifdef DEBUG_ISP
        SMoDebug.print(ix == responseIndex ? " ![" : " [");
        SMoDebug.print(recv, HEX);
        SMoDebug.print("] ");
#endif
        if (ix == responseIndex)
            response = recv;
    }
#ifdef DEBUG_ISP
    SMoDebug.println();
#endif
    return response;
}

static uint8_t
SPITransaction(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4)
{
#ifdef DEBUG_ISP
    SMoDebug.print("SPI ");
    SMoDebug.print(b1, HEX);
    SMoDebug.print(" ");
    SMoDebug.print(b2, HEX);
    SMoDebug.print(" ");
    SMoDebug.print(b3, HEX);
    SMoDebug.print(" ");
    SMoDebug.print(b4, HEX);
#endif
    SPITransfer(b1);
    SPITransfer(b2);
    SPITransfer(b3);
#ifdef DEBUG_ISP
    uint8_t result = SPITransfer(b4);
    SMoDebug.print(" [");
    SMoDebug.print(result, HEX);
    SMoDebug.println("]");
  
    return result;  
#else
    return SPITransfer(b4);
#endif
}

static bool
ISPPollReady()
{
    uint32_t time = millis();
    while (!SPITransaction(0xF0, 0, 0, 0)) // Poll RDY/BSY
        if (millis() - time > DEFAULTTIMEOUT)
            return false;
    return true;
}

void
SMoISP::EnterProgmode()
{
#ifdef DEBUG_ISP
    SMoDebugInit();
    // SMoDebug.print("Pin layout ");
    // SMoDebug.print(SMO_LAYOUT);
    SMoDebug.print(" RESET ");
    SMoDebug.println(ISP_RESET);
#endif
    // const uint8_t   timeOut     =   SMoCommand::gBody[1];
    const uint8_t   stabDelay   =   SMoCommand::gBody[2];
    // const uint8_t   cmdexeDelay =   SMoCommand::gBody[3];
    // const uint8_t   synchLoops  =   SMoCommand::gBody[4];
    // const uint8_t   byteDelay   =   SMoCommand::gBody[5];
    const uint8_t   pollValue   =   SMoCommand::gBody[6];
    const uint8_t   pollIndex   =   SMoCommand::gBody[7];
    const uint8_t * command     =  &SMoCommand::gBody[8];

    //
    // Set up SPI
    //
#if SMO_LAYOUT==SMO_LAYOUT_HVPROG2
    // make sure that 12V regulator is shutdown 
    digitalWrite(SMO_HVENABLE, LOW);
    pinMode(SMO_HVENABLE, OUTPUT);
#endif
    // to avoid glitch in SCK and MOSI we must set CPOL before SPI.begin()
    SPCR &= ~_BV(CPOL) & ~_BV(CPHA); // idle LOW for SPI_MODE0
    //
    SPI.begin();
    SPI.beginTransaction(SMoGeneral::gSCKDuration > 2 ? ISPSPISetting[2] : ISPSPISetting[SMoGeneral::gSCKDuration]);
    ISPAssertReset();
    pinMode(ISP_RESET, OUTPUT);
    delay(stabDelay);

    //
    // Set up clock generator on OC1A (OC2A or OC2 if HVPROG2)
    //
    pinMode(MCU_CLOCK, OUTPUT);
#if SMO_LAYOUT==SMO_LAYOUT_HVPROG2
#ifdef TCCR2
    TCCR2  = _BV(COM20) | _BV(WGM21);  // Stop Timer 2
    TCNT2  = 0xFF;                     // Initialize counter value
    OCR2   = SMoGeneral::gClockMatch;  // Set compare match value
    // CTC mode, Toggle OC2 on Compare Match. Set timer operation mode and prescaler
    TCCR2  = _BV(COM20) | _BV(WGM21) | (0x07 & SMoGeneral::gPrescale);
#else
    TCCR2B = 0;                        // Stop Timer 2
    TCCR2A = _BV(COM2A0) | _BV(WGM21); // CTC mode, Toggle OC2A on Compare Match
    TCNT2  = 0xFF;                     // Initialize counter value
    OCR2A  = SMoGeneral::gClockMatch;  // Set compare match value
    TCCR2B = 0x07 & SMoGeneral::gPrescale;// Set timer operation mode and prescaler
#endif
#else
    TCCR1B = 0;                        // Stop clock generator
    TCCR1A = _BV(COM1A0);              // CTC mode, toggle OC1A on comparison with OCR1A
    OCR1A  = 0;                        // F(OC1A) = 16MHz / (2*8*(1+0)) == 1MHz
    TIMSK1 = 0;
    TCCR1B = _BV(WGM12) | _BV(CS11);   // Prescale by 8
    TCNT1  = 0;
#endif
    
    //
    // Now issue the programming mode instruction
    //
    digitalWrite(SCK, LOW);

    uint8_t response = SPITransaction(command, pollIndex-1);
    if (response != pollValue) {
        //
        // Ooops, that's bad. Try again in limp mode
        //
        SPI.endTransaction();
        SPI.end();       
        sSPILimpMode = 2;   // Start at 16µs, 60kHz bit clock
        pinMode(MOSI, OUTPUT);
        pinMode(SCK, OUTPUT);
        pinMode(MISO, INPUT);
        
        do {
#ifdef DEBUG_ISP
            SMoDebug.print("Retrying in limp mode ");
            SMoDebug.print(sSPILimpMode);
            SMoDebug.print(" (");
            SMoDebug.print(1000.0 / (4 << sSPILimpMode));
            SMoDebug.println("kHz).");
#endif
            ISPDeassertReset();
            digitalWrite(SCK, LOW);
            delay(50);
            ISPAssertReset();
            delay(50);
            response     = SPITransaction(command, pollIndex-1);
        } while (response != pollValue && sSPILimpMode++ < kMaxLimp);
    }
    SMoCommand::SendResponse(response==pollValue ? STATUS_CMD_OK : STATUS_CMD_FAILED);
}

void
SMoISP::LeaveProgmode()
{
    const uint8_t   preDelay  =   SMoCommand::gBody[1];
    const uint8_t   postDelay  =   SMoCommand::gBody[2];  

    if (sSPILimpMode)
        sSPILimpMode = false;
    else {
        SPI.endTransaction();
        SPI.end();     // Stop SPI
    }
    delay(preDelay);
    ISPDeassertReset();
// stop timer
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
    delay(postDelay);
    SMoCommand::SendResponse();
}

void
SMoISP::ChipErase()
{
    const uint8_t   eraseDelay  =   SMoCommand::gBody[1];
    const uint8_t   pollMethod  =   SMoCommand::gBody[2];
    const uint8_t * command     =  &SMoCommand::gBody[3];    

    SPITransaction(command);
    if (pollMethod) {
        if (!ISPPollReady()) {
            SMoCommand::SendResponse(STATUS_RDY_BSY_TOUT);
            return;
        }
    } else
        delay(eraseDelay);
    SMoCommand::SendResponse();
}

static void
ProgramMemory(bool flash)
{
    uint16_t  numBytes          = SMoCommand::gBody[1] << 8 | SMoCommand::gBody[2];
    uint8_t   mode              = SMoCommand::gBody[3];
    const uint8_t   cmdDelay    = SMoCommand::gBody[4];
    const uint8_t   cmd1        = SMoCommand::gBody[5];
    const uint8_t   cmd2        = SMoCommand::gBody[6];
    const uint8_t   cmd3        = SMoCommand::gBody[7];
    const uint8_t   pollVal1    = SMoCommand::gBody[8];
    const uint8_t   pollVal2    = SMoCommand::gBody[9];
    const uint8_t * data        = &SMoCommand::gBody[10];

    uint8_t pollVal = flash ? pollVal1 : pollVal2;
    if (SMoGeneral::gAddress.d.extH & 0x80)
        SPITransaction(0x4D, 0, SMoGeneral::gAddress.d.extL, 0); // Load Extended Address byte
    do {
        SPITransaction(!(numBytes & 1) || !flash ? cmd1 : cmd1|8, SMoGeneral::gAddress.c[1], SMoGeneral::gAddress.c[0], *data);
        if (!(mode & 0x01) || (numBytes == 1 && mode & 0x80)) { // Byte mode or page loaded
            if (mode & 0x01) // Page mode
                SPITransaction(cmd2, SMoGeneral::gAddress.c[1], SMoGeneral::gAddress.c[0], 0);
            if (mode & 0x12) // Timed delay
                delay(cmdDelay);
            else if (mode & 0x24) { // Value polling
                if (pollVal == *data)
                    delay(cmdDelay); // Values are identical - don't poll
                else
                    while (SPITransaction(!(numBytes & 1) || !flash ? cmd3 : cmd3|8, SMoGeneral::gAddress.c[1], SMoGeneral::gAddress.c[0], 0) == pollVal)
                        ;
            } else if (mode & 0x48 && !ISPPollReady()) // RDY/BSY polling
                goto TIMEOUT_ProgramMemory;
        }
        data++;
        if (!(--numBytes & 1) || !flash) {
            SMoGeneral::gAddress.d.addr++;
            if (!numBytes)
                break;
            if (SMoGeneral::gAddress.d.addr == 0)
                SPITransaction(0x4D, 0, ++SMoGeneral::gAddress.d.extL, 0); // Load Extended Address byte
        }
    } while (numBytes);
    SMoCommand::SendResponse();
    return;
TIMEOUT_ProgramMemory:
    SMoCommand::SendResponse(STATUS_RDY_BSY_TOUT);
}

static void
ReadMemory(bool flash)
{
    uint16_t  numBytes    = SMoCommand::gBody[1] << 8 | SMoCommand::gBody[2];
    const uint8_t   cmd   = SMoCommand::gBody[3];
    uint8_t *outData      = &SMoCommand::gBody[2];

    if (SMoGeneral::gAddress.d.extH & 0x80)
        SPITransaction(0x4D, 0, SMoGeneral::gAddress.d.extL, 0); // Load Extended Address byte
    do {
        *outData++ = SPITransaction(!(numBytes & 1) || !flash ? cmd : cmd|8, SMoGeneral::gAddress.c[1], SMoGeneral::gAddress.c[0], 0);
        if (!(--numBytes & 1) || !flash) {
            SMoGeneral::gAddress.d.addr++;
            if (!numBytes)
                break;
            if (SMoGeneral::gAddress.c[0] == 0)
                SPITransaction(0x4D, 0, ++SMoGeneral::gAddress.d.extL, 0); // Load Extended Address byte
        }
    } while (numBytes);
    *outData++ = STATUS_CMD_OK;
    SMoCommand::SendResponse(STATUS_CMD_OK, outData - &SMoCommand::gBody[0]);   
}

void
SMoISP::ProgramFlash()
{
    ProgramMemory(true);
}

void
SMoISP::ReadFlash()
{
    ReadMemory(true);
}

void
SMoISP::ProgramEEPROM()
{
    ProgramMemory(false);
}

void
SMoISP::ReadEEPROM()
{
    ReadMemory(false);
}

void
SMoISP::ProgramFuse()
{
    SPITransaction(&SMoCommand::gBody[1]);
    SMoCommand::gBody[2] = STATUS_CMD_OK;
    SMoCommand::SendResponse();
}

void
SMoISP::ReadFuse()
{
    uint8_t pollIndex   = SMoCommand::gBody[1];
    SMoCommand::gBody[2]= SPITransaction(&SMoCommand::gBody[2], pollIndex-1);
    SMoCommand::gBody[3] = STATUS_CMD_OK;
    SMoCommand::SendResponse(STATUS_CMD_OK, 4);
}

void
SMoISP::SPIMulti()
{
    uint8_t         numTX   =   SMoCommand::gBody[1];
    uint8_t         numRX   =   SMoCommand::gBody[2];
    uint8_t         rxStart =   SMoCommand::gBody[3];
    const uint8_t * txData  =  &SMoCommand::gBody[4];
    uint8_t *       rxData  =  &SMoCommand::gBody[2];

#ifdef DEBUG_ISP
    SMoDebug.print("!SPI");
#endif
    while (numTX) {
#ifdef DEBUG_ISP
        SMoDebug.print(" ");
        SMoDebug.print(*txData, HEX);
#endif
        *rxData = SPITransfer(*txData++);
        --numTX;
#ifdef DEBUG_ISP
        SMoDebug.print(rxStart ? " (" : " [");
        SMoDebug.print(*rxData, HEX);
        SMoDebug.print(rxStart ? ")" : "]");
#endif
        if (rxStart) {
            --rxStart;
        } else if (numRX) {
            ++rxData;
            --numRX;
        }
    }
    while (numRX) {
        *rxData = SPITransfer(0);
#ifdef DEBUG_ISP
        SMoDebug.print(rxStart ? " . (" : " . [");
        SMoDebug.print(*rxData, HEX);
        SMoDebug.print(rxStart ? ")" : "]");
#endif
        if (rxStart) {
            --rxStart;
        } else {
            ++rxData;
            --numRX;
        }
    }
    *rxData++ = STATUS_CMD_OK;
    SMoCommand::SendResponse(STATUS_CMD_OK, rxData-&SMoCommand::gBody[0]);
#ifdef DEBUG_ISP
    SMoDebug.println();
#endif
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
