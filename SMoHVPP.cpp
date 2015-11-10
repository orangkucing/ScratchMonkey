// -*- mode: c++; tab-width: 4; indent-tabs-mode: nil -*-
//
// ScratchMonkey 0.1        - STK500v2 compatible programmer for Arduino
//
// File: SMoHVPP.cpp        - High Voltage Parallel Programming
//                            (for MCUs with 20 pins and more)
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

#include "SMoHVPP.h"
#include "SMoCommand.h"
#include "SMoGeneral.h"
#include "SMoConfig.h"

#ifdef DEBUG_HVPP
#include "SMoDebug.h"
#endif

#include <Arduino.h>
#include <SPI.h>

enum {
    HVPP_RESET  = SMO_HVRESET,
    HVPP_VCC    = SMO_SVCC,
#if SMO_LAYOUT==SMO_LAYOUT_STANDARD
    HVPP_RCLK   = A1,
    HVPP_XTAL   = A2,
#define HVPP_TOGGLE_XTAL    do { PORTC |= _BV(2); PORTC &= ~_BV(2); } while (0)
#define HVPP_RDY  12
#define ISREADY    (digitalRead(HVPP_RDY))
#elif SMO_LAYOUT==SMO_LAYOUT_LEONARDO
    HVPP_XTAL   = 13,
#define HVPP_TOGGLE_XTAL    do { PORTC |= _BV(7); PORTC &= ~_BV(7); } while (0)
#define HVPP_RDY  12
#define ISREADY    (digitalRead(HVPP_RDY))
#elif SMO_LAYOUT==SMO_LAYOUT_MEGA
    HVPP_XTAL   = 13,
#define HVPP_TOGGLE_XTAL    do { PORTB |= _BV(7); PORTB &= ~_BV(7); } while (0)
#define HVPP_RDY  12
#define ISREADY    (digitalRead(HVPP_RDY))
#elif SMO_LAYOUT==SMO_LAYOUT_HVPROG2
    HVPP_XTAL   = 15,
#define HVPP_TOGGLE_XTAL    do { PORTD &= ~_BV(7); PORTD |= _BV(7); } while (0)
#define ISREADY    (PINC & _BV(SMoGeneral::gControlStack[kRdyBsyBit])) 
#endif
};

//
// HVPP, for 28 pins and more, requires 8 output signals and 1 input 
// signal. 20 pin MCUs multiplex some of the output signals, and they
// don't do it all the same way. We rely on the control stack uploaded
// by avrdude to tell us what signals to set, when. 
//
// This implementation uses a 74HV595 shift register for the output 
// signal, but if you're using a board with more output pins, or are
// willing to have the TX/RX pins do double duty, you may be able to 
// dispense with that.
//
// Control patterns, expressed as index values for the control stack
// (Adopted from Simon Quiang's AVRminiProg implementation)
//
enum {
    kLoadAddr       =  0,
    kLoadData       =  4,
    kLoadCommand    =  8,
    kDone           = 12,
    kCommitData     = 16,
    kEnableRead     = 20,

    kPageLoad       = 24,
    kRdyBsyMask     = 25,
    kOEdelay        = 26, // Atmel's newest firmware 2.10 seems to ignore this value
    kRdyBsyBit      = 27,
    kInit           = 28,
//                           0x00
//                           0x00
    kPoll           = 31, // 0x00 for all but 0x01 CAN32/64/128 and 4414/4434, 0x02 m644/1284 family and m2560/2561

    kLowByte        = 0,
    kHighByte       = 1,
    kExtByte        = 2,
    kExt2Byte       = 3,
};

#define HVPPControlPattern(c, b) (SMoGeneral::gControlStack[c + b])

// Command Byte Bit Coding
#define HVPP_FLASH           0x00
#define HVPP_EEPROM          0x01

#define HVPP_ChipErase       0x80
#define HVPP_WriteFuseBits   0x40
#define HVPP_WriteLockBits   0x20
#define HVPP_WriteMemory     0x10
#define HVPP_WriteFlash      (HVPP_WriteMemory | HVPP_FLASH)
#define HVPP_WriteEEPROM     (HVPP_WriteMemory | HVPP_EEPROM)
#define HVPP_ReadSignature   0x08
#define HVPP_ReadFuseLock    0x04
#define HVPP_ReadMemory      0x02
#define HVPP_ReadFlash       (HVPP_ReadMemory | HVPP_FLASH)
#define HVPP_ReadEEPROM      (HVPP_ReadMemory | HVPP_EEPROM)
#define HVPP_NoOperation     0x00
//
// Control/Data access
//
#if SMO_LAYOUT==SMO_LAYOUT_STANDARD
//
// Delegate controls to auxiliary 74HC595 shift register, but
// can transfer data pretty easily
//
enum {
    PORTD_MASK = 0xFC,
    PORTB_MASK = 0x03,
    PORTD_SHIFT = 2,
    PORTB_SHIFT = 6
};

inline void
HVPPSetControlSignals(uint8_t signals)
{
    digitalWrite(HVPP_RCLK, LOW);
    SPI.transfer(signals);
    digitalWrite(HVPP_RCLK, HIGH);
}

inline void
HVPPInitControlSignals()
{
    SPI.begin();
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV2);// Pedal to the metal
    digitalWrite(HVPP_RCLK, LOW);
    pinMode(HVPP_RCLK, OUTPUT);
    pinMode(HVPP_RDY, INPUT);
}

inline void
HVPPEndControls()
{
    SPI.end();
}

inline void
HVPPSetDataMode(uint8_t mode)
{
    if (mode == OUTPUT) {
        DDRD |= PORTD_MASK;
        DDRB |= PORTB_MASK;
    } else {
        DDRD &= ~PORTD_MASK;
        DDRB &= ~PORTB_MASK;
    }
}

inline void
HVPPSetDataBits(uint8_t dataOut)
{
    PORTD = (PORTD & ~PORTD_MASK) | ((dataOut << PORTD_SHIFT) & PORTD_MASK);
    PORTB = (PORTB & ~PORTB_MASK) | ((dataOut >> PORTB_SHIFT) & PORTB_MASK);
}

inline uint8_t
HVPPGetDataBits()
{
    // No need for masking
    return ((PINB << PORTB_SHIFT) | (PIND >> PORTD_SHIFT)) & 0xFF;
}
#elif SMO_LAYOUT==SMO_LAYOUT_LEONARDO
//
// Leonardos don't have 8 contiguous pins anywhere, so we split the 
// control signals across two ports. The data signals are not as 
// critical, so we just use digitalRead (we'd have to split them
// across at least three ports).
//
enum {
    PORTF_MASK = 0xF1,
    PORTD_MASK = 0x0C,
};

inline void
HVPPSetControlSignals(uint8_t signals)
{
    PORTF   = (PORTF & ~PORTF_MASK) | (signals & PORTF_MASK);
    PORTD   = (PORTD & ~PORTD_MASK) | (signals & PORTD_MASK);
}

inline void
HVPPInitControlSignals()
{
    DDRF   |= PORTF_MASK;
    DDRD   |= PORTD_MASK;
    pinMode(HVPP_RDY, INPUT);
}

inline void
HVPPEndControls()
{
}

inline void
HVPPSetDataMode(uint8_t mode)
{
    for (uint8_t pin=2; pin<10; ++pin)
        pinMode(pin, mode);
}

inline void
HVPPSetDataBits(uint8_t dataOut)
{
    for (uint8_t pin=2; pin<10; ++pin) {
        digitalWrite(pin, dataOut & 1);
        dataOut >>= 1;
    }
}

inline uint8_t
HVPPGetDataBits()
{
    uint8_t dataIn;

    for (uint8_t pin=9; pin >= 2; --pin)
        dataIn = (dataIn << 1) | digitalRead(pin);

    return dataIn;
}
#elif SMO_LAYOUT==SMO_LAYOUT_MEGA
//
// Megas have lots of contiguous pins, so we just use two full ports.
//
inline void
HVPPSetControlSignals(uint8_t signals)
{
    PORTF   = signals;
}

inline void
HVPPInitControlSignals()
{
    DDRF    = 0xFF;
    pinMode(HVPP_RDY, INPUT);
}

inline void
HVPPEndControls()
{
}

inline void
HVPPSetDataMode(uint8_t mode)
{
    if (mode == OUTPUT) {
        DDRK = 0xFF;
    } else {
        DDRK = 0x00;
    }
}

inline void
HVPPSetDataBits(uint8_t dataOut)
{
    PORTK = dataOut;
}

inline uint8_t
HVPPGetDataBits()
{
    return PINK;
}
#elif SMO_LAYOUT==SMO_LAYOUT_HVPROG2
//
// DIP-40 AVRs have enough numbers of contiguous pins
//
inline void
HVPPSetControlSignals(uint8_t signals)
{
    PORTC = signals & SMoGeneral::gControlStack[kRdyBsyMask];
}

inline void
HVPPInitControlSignals()
{
    // disable pullups as SPI.end() doesn't restore them
    DDRB = 0; PORTB = 0; DDRB = 0xFF;
    // set data directions
    DDRC = SMoGeneral::gControlStack[kRdyBsyMask];
}

inline void
HVPPEndControls()
{
}

inline void
HVPPSetDataMode(uint8_t mode)
{
    if (mode == OUTPUT)
        DDRB = 0xFF;
    else
        DDRB = 0x00;
}

inline void
HVPPSetDataBits(uint8_t dataOut)
{
    PORTB = dataOut;
}

inline uint8_t
HVPPGetDataBits()
{
    return PINB;
}
#endif

static void
HVPPControls(uint8_t c)
{
#ifdef DEBUG_HVPP
    SMoDebug.print("Ctrl ");
    SMoDebug.print(c, BIN);
    SMoDebug.println();
#endif
    HVPPSetControlSignals(c);
}

static void
HVPPControls(uint8_t c, uint8_t data)
{
#ifdef DEBUG_HVPP
    SMoDebug.print("Ctrl ");
    SMoDebug.print(c, BIN);
    SMoDebug.println();
#endif
    HVPPSetControlSignals(c);
#ifdef DEBUG_HVPP
    SMoDebug.print("Data<");
    SMoDebug.println(data, HEX);
#endif
    HVPPSetDataBits(data);
    HVPP_TOGGLE_XTAL;
}

static void
HVPPControls(uint8_t c, uint8_t *p)
{
#ifdef DEBUG_HVPP
    SMoDebug.print("Ctrl ");
    SMoDebug.print(c, BIN);
    SMoDebug.println();
#endif
    HVPPSetControlSignals(c);
    // very short delay for t2313A
    switch (SMoGeneral::gControlStack[kOEdelay]) {
    case 4:
        __asm__ __volatile__ ("nop\n\t");
    case 3:
        __asm__ __volatile__ ("nop\n\t");
    case 2:
        __asm__ __volatile__ ("nop\n\t");
    case 1:
        __asm__ __volatile__ ("nop\n\t");
    default:
        break;
    }
    *p = HVPPGetDataBits();
#ifdef DEBUG_HVPP
    SMoDebug.print("Data>");
    SMoDebug.println(*p, HEX);
#endif
}

inline void
HVPPInitControls()
{
    HVPPInitControlSignals();
    HVPPSetControlSignals(HVPPControlPattern(kInit, kLowByte));            // Set all control pins to zero
}

static bool
HVPPPollWait(uint8_t pollTimeout)
{
    uint32_t time = millis();
    delayMicroseconds(1);
    while (!ISREADY)
        if (millis() - time > (uint32_t)pollTimeout)
            return false;
    return true;
}

void
SMoHVPP::EnterProgmode()
{
#ifdef DEBUG_HVPP
    SMoDebugInit();
#endif

    const uint8_t   stabDelay   = SMoCommand::gBody[1];
    const uint8_t   progModeDelay = SMoCommand::gBody[2];
    const uint8_t   latchCycles = SMoCommand::gBody[3];
    const uint8_t   toggleVtg   = SMoCommand::gBody[4];
    const uint8_t   powerOffDelay = SMoCommand::gBody[5];
    const uint8_t   resetDelayMs = SMoCommand::gBody[6];
    const uint8_t   resetDelayUs = SMoCommand::gBody[7];

    delay(progModeDelay);
    // power off target
    digitalWrite(HVPP_VCC, LOW);
    pinMode(HVPP_VCC, OUTPUT);
    delayMicroseconds(250);
    // target RESET = 0V
    digitalWrite(HVPP_RESET, HIGH);
    pinMode(HVPP_RESET, OUTPUT);
    // set control pins
    HVPPInitControls();
    HVPP_TOGGLE_XTAL;
    pinMode(HVPP_XTAL, OUTPUT);
#if SMO_LAYOUT==SMO_LAYOUT_HVPROG2
    digitalWrite(SMO_HVENABLE, HIGH);
    pinMode(SMO_HVENABLE, OUTPUT); // enable 12V
#endif
    // make sure HVPP_VCC is 0V
    delay(powerOffDelay);
    if (toggleVtg) {
#if defined(SMO_AVCC)
        uint32_t time = millis();
        while (analogRead(SMO_AVCC) > 50) {   // wait until HVPP_VCC become lower than 0.3V
            if (millis() - time > 100)  // timeout
                break;
        }
#else
        delay(100);
#endif
    }
    // power on target
    digitalWrite(HVPP_VCC, HIGH);
    delay(resetDelayMs);
    delayMicroseconds(resetDelayUs * 10 + 250); // add extra 250 microseconds
    // toggle XTAL1
    for (uint8_t i=0; i<latchCycles; ++i) {
        HVPP_TOGGLE_XTAL;
    }
    // apply 12V to RESET
    digitalWrite(HVPP_RESET, LOW);
    delay(stabDelay);

    HVPPSetDataMode(OUTPUT);
    HVPPControls(HVPPControlPattern(kDone, kLowByte));
    SMoCommand::SendResponse();
}

void
SMoHVPP::LeaveProgmode()
{
    const uint8_t   stabDelay  = SMoCommand::gBody[1];
    const uint8_t   resetDelay = SMoCommand::gBody[2];

    digitalWrite(HVPP_VCC, LOW);
    digitalWrite(HVPP_RESET, HIGH);
    delay(resetDelay);

#if SMO_LAYOUT==SMO_LAYOUT_HVPROG2
    digitalWrite(SMO_HVENABLE, LOW); // disable 12V
    digitalWrite(HVPP_VCC, HIGH);
    digitalWrite(HVPP_RESET, LOW);
#endif
    delay(stabDelay);
    SMoCommand::SendResponse();
}

void
SMoHVPP::ChipErase()
{
    const uint8_t pulseWidth    = SMoCommand::gBody[1];
    const uint8_t pollTimeout   = SMoCommand::gBody[2];

    HVPPControls(HVPPControlPattern(kLoadCommand, kLowByte), (uint8_t)HVPP_ChipErase);
    HVPPControls(HVPPControlPattern(kCommitData, kLowByte));
    delay(pulseWidth);
    HVPPControls(HVPPControlPattern(kDone, kLowByte));
    // SMoGeneral::gControlStack[kPoll] is simply ignored. :)
    if (pollTimeout)
        HVPPPollWait(pollTimeout);
    SMoCommand::SendResponse();
}

static void
ProgramMemory(bool flash)
{
    uint16_t        numBytes    =  (SMoCommand::gBody[1] << 8) | SMoCommand::gBody[2];
    const uint8_t   mode        =   SMoCommand::gBody[3];
    const uint8_t   pollTimeout =   SMoCommand::gBody[4];
    const uint8_t * data        =  &SMoCommand::gBody[5];

    uint16_t pageMask = (1 << ((mode & 0x0E ? mode & 0x0E : 0x10) >> 1) - (flash ? 1 : 0)) - 1;
    int8_t b;
    HVPPControls(HVPPControlPattern(kLoadCommand, kLowByte), (uint8_t)(flash ? HVPP_WriteFlash : HVPP_WriteEEPROM));
    if (SMoGeneral::gAddress.d.extH & 0x80)
        HVPPControls(HVPPControlPattern(kLoadAddr, kExtByte), SMoGeneral::gAddress.d.extL);
    HVPPControls(HVPPControlPattern(kLoadAddr, kHighByte), SMoGeneral::gAddress.c[1]);
    HVPPControls(HVPPControlPattern(kLoadAddr, kLowByte), SMoGeneral::gAddress.c[0]);    
    do {
        b = (!(numBytes & 1) || !flash) ? kLowByte : kHighByte; 
        HVPPControls(HVPPControlPattern(kLoadData, b), *data);
        if (!(mode & 1)) { // Byte mode
            HVPPControls(HVPPControlPattern(kCommitData, b));
            HVPPControls(HVPPControlPattern(kDone, b));
            if (!HVPPPollWait(pollTimeout))
                goto TIMEOUT_ProgramMemory;              
        }
        data++;
        if (!(--numBytes & 1) || !flash) {
            SMoGeneral::gAddress.d.addr++;
            if (mode & 1) { // Page mode
                HVPPControls(HVPPControlPattern(kPageLoad, kLowByte)); // assert PAGEL
                if (!(SMoGeneral::gAddress.d.addr & pageMask) || (numBytes == 0 && mode & 0xC0)) { // Write page to memory
                    HVPPControls(HVPPControlPattern(kCommitData, kLowByte));
                    HVPPControls(HVPPControlPattern(kDone, kLowByte));
                    if (!HVPPPollWait(pollTimeout))
                        goto TIMEOUT_ProgramMemory;
                }
            }
            if (!numBytes)
                break;
            if (SMoGeneral::gAddress.c[0] == 0) {
                if (SMoGeneral::gAddress.c[1] == 0)
                    HVPPControls(HVPPControlPattern(kLoadAddr, kExtByte), ++SMoGeneral::gAddress.d.extL);
                HVPPControls(HVPPControlPattern(kLoadAddr, kHighByte), SMoGeneral::gAddress.c[1]);
            }
            HVPPControls(HVPPControlPattern(kLoadAddr, kLowByte), SMoGeneral::gAddress.c[0]);
        }
    } while (numBytes);
    if (mode & 0x40)
        HVPPControls(HVPPControlPattern(kLoadCommand, kLowByte), (uint8_t)HVPP_NoOperation);
    SMoCommand::SendResponse();
    return;
TIMEOUT_ProgramMemory:
    SMoCommand::SendResponse(STATUS_RDY_BSY_TOUT);
}

static void
ReadMemory(bool flash)
{
    uint16_t    numBytes    =  (SMoCommand::gBody[1] << 8) | SMoCommand::gBody[2];
    uint8_t *   dataOut     =  &SMoCommand::gBody[2];
    
    int8_t b;
    HVPPControls(HVPPControlPattern(kLoadCommand, kLowByte), (uint8_t)(flash ? HVPP_ReadFlash : HVPP_ReadEEPROM));
    if (SMoGeneral::gAddress.d.extH & 0x80)
        HVPPControls(HVPPControlPattern(kLoadAddr, kExtByte), SMoGeneral::gAddress.d.extL);
    HVPPControls(HVPPControlPattern(kLoadAddr, kHighByte), SMoGeneral::gAddress.c[1]);
    HVPPControls(HVPPControlPattern(kLoadAddr, kLowByte), SMoGeneral::gAddress.c[0]);
    HVPPSetDataMode(INPUT);
    do {
        b = (!(numBytes & 1) || !flash) ? kLowByte : kHighByte;    
        HVPPControls(HVPPControlPattern(kEnableRead, b), (uint8_t *)dataOut++);
        if (!(--numBytes & 1) || !flash) {
            HVPPControls(HVPPControlPattern(kDone, b));
            SMoGeneral::gAddress.d.addr++;
            if (!numBytes)
                break;
            HVPPSetDataMode(OUTPUT);
            if (SMoGeneral::gAddress.c[0] == 0) {
                if (SMoGeneral::gAddress.c[1] == 0)
                    HVPPControls(HVPPControlPattern(kLoadAddr, kExtByte), ++SMoGeneral::gAddress.d.extL);
                HVPPControls(HVPPControlPattern(kLoadAddr, kHighByte), SMoGeneral::gAddress.c[1]);
            }
            HVPPControls(HVPPControlPattern(kLoadAddr, kLowByte), SMoGeneral::gAddress.c[0]);
            HVPPSetDataMode(INPUT);
        }
    } while (numBytes);
    *dataOut++ = STATUS_CMD_OK;
    SMoCommand::SendResponse(STATUS_CMD_OK, dataOut - &SMoCommand::gBody[0]);
    HVPPSetDataMode(OUTPUT);
}

void
SMoHVPP::ProgramFlash()
{
    ProgramMemory(true);
}

void
SMoHVPP::ReadFlash()
{
    ReadMemory(true);
}

void
SMoHVPP::ProgramEEPROM()
{
    ProgramMemory(false);
}

void
SMoHVPP::ReadEEPROM()
{
    ReadMemory(false);
}

static void 
ProgramFuseLock(uint8_t command, uint8_t byteSel)
{
    const uint8_t   value       = SMoCommand::gBody[2];
    const uint8_t   pulseWidth  = SMoCommand::gBody[3];
    const uint8_t   pollTimeout = SMoCommand::gBody[4];

    HVPPControls(HVPPControlPattern(kLoadCommand, kLowByte), command);
    HVPPControls(HVPPControlPattern(kLoadData, kLowByte), value);
    HVPPControls(HVPPControlPattern(kCommitData, byteSel));
    delay(pulseWidth);
    HVPPControls(HVPPControlPattern(kDone, byteSel));
    // AT90S1200 doesn't generate any activity on RDY/BSY pin.
    // So just ignore the return value even if timeout occurs.
    HVPPPollWait(pollTimeout);
    SMoCommand::SendResponse(STATUS_CMD_OK);
}

static void
ReadFuseLock(uint8_t byteSel)
{
    uint8_t * dataOut   = &SMoCommand::gBody[2];

    HVPPControls(HVPPControlPattern(kLoadCommand, kLowByte), (uint8_t)HVPP_ReadFuseLock);
    HVPPSetDataMode(INPUT);
    HVPPControls(HVPPControlPattern(kEnableRead, byteSel), (uint8_t *)dataOut);
    HVPPControls(HVPPControlPattern(kDone, byteSel));

    SMoCommand::SendResponse(STATUS_CMD_OK, 3);
    HVPPSetDataMode(OUTPUT);
}

void
SMoHVPP::ProgramFuse()
{
    const uint8_t   byteSel        = SMoCommand::gBody[1];

    ProgramFuseLock(HVPP_WriteFuseBits, byteSel);
}

void
SMoHVPP::ReadFuse()
{
    const uint8_t   byteSel    = SMoCommand::gBody[1];

    ReadFuseLock(byteSel == kHighByte ? kExt2Byte : byteSel);
}

void
SMoHVPP::ProgramLock()   
{
    ProgramFuseLock(HVPP_WriteLockBits, kLowByte);
}

void
SMoHVPP::ReadLock()       
{
    ReadFuseLock(kHighByte);
}

static void
ReadSignatureCal(uint8_t byteSel, uint8_t addr)
{
    uint8_t * dataOut   = &SMoCommand::gBody[2];

    HVPPControls(HVPPControlPattern(kLoadCommand, kLowByte), (uint8_t)HVPP_ReadSignature);
    HVPPControls(HVPPControlPattern(kLoadAddr, kLowByte), addr);
    HVPPSetDataMode(INPUT);
    HVPPControls(HVPPControlPattern(kEnableRead, byteSel), (uint8_t *)dataOut);
    HVPPControlPattern(kDone, byteSel);

    SMoCommand::SendResponse(STATUS_CMD_OK, 3);
    HVPPSetDataMode(OUTPUT);
}

void
SMoHVPP::ReadSignature()  
{
    const uint8_t   addr    = SMoCommand::gBody[1];

    ReadSignatureCal(kLowByte, addr);
}

void
SMoHVPP::ReadOscCal()     
{
    ReadSignatureCal(kHighByte, 0x00);
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
