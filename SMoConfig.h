// -*- mode: c++; tab-width: 4; indent-tabs-mode: nil -*-
//
// ScratchMonkey 0.1        - STK500v2 compatible programmer for Arduino
//
// File: SMoConfig.h        - Configuration options
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

#ifndef _SMO_CONFIG_
#define _SMO_CONFIG_

#include "Arduino.h"

//
// We support a number of different pin layouts:
//  - Standard Arduino: SPI on pins 10-13, pins 0/1 used for Serial (ATmega168/328)
//  - Leonardo/Micro:   SPI on dedicated pins, pins 0/1 available   (ATmega32u4)
//  - Mega:             SPI on pins 50-53, pins 0/1 used for Serial (ATmega1280/2560)
//  - HVprog2:          Atmel's original STK500 compatible          (DIP-40 AVRs)
//

#define    SMO_LAYOUT_STANDARD     0
#define    SMO_LAYOUT_LEONARDO     1
#define    SMO_LAYOUT_MEGA         2
#define    SMO_LAYOUT_HVPROG2      3

// A 5V-logic Arduino needs a 5V-tolerant bus buffer 74LVxxx125 as a level shifter to do PDI programming where the target voltage must be 3.3V.
// For this purpose no automatic level shifter is known to work stably, i.e., none of MAX3002, GTL2003, TXB0104, or FXMA108 works reliably.
// cf. Adding 100 kilo ohm resistor between PDI_DATA and GND sometimes solves the problem:
//            http://blog.frankvh.com/2009/09/22/avr-xmega-and-avrisp-mk2/
//            http://www.avrfreaks.net/forum/intermittant-programming-xplained-board-avrisp2?name=PNphpBB2&file=viewtopic&t=107517
//     Or adding about 100 pico farad capacitor also helps 
#if defined(__AVR_ATmega32U4__)
#define SMO_LAYOUT  SMO_LAYOUT_LEONARDO
// Since Leonardo is 5V logic we must use a 5V tolerant bus buffer 74LVxxx125 as a level shifter to PDI programming voltage 3.3V.
#define MOSI_GATE 8
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define SMO_LAYOUT  SMO_LAYOUT_MEGA
// Since Mega is 5V logic we must use a 5V tolerant bus buffer 74LVxxx125 as a level shifter to PDI programming voltage 3.3V.
#define MOSI_GATE 8
#elif defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega168P__)
#define SMO_LAYOUT  SMO_LAYOUT_STANDARD
// Since Uno is 5V logic we must use a 5V tolerant bus buffer 74LVxxx125 as a level shifter to PDI programming voltage 3.3V.
// If you are using a 3.3V logic such as Pro Mini 3.3V 8MHz then comment out the following line. 
#define MOSI_GATE 8
#elif defined(__AVR_ATmega16__) || defined(__AVR_ATmega164P__) || defined(__AVR_ATmega32__) || defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)
#define SMO_LAYOUT  SMO_LAYOUT_HVPROG2
// HVPROG2 can run the board in 3.3V logic by supplying 3.3V to its VCC. In this case no bus buffer is necessary for PDI programming.
// But Atmel's original STK500 board needs a 5V tolerant bus buffer 74LVxxx125 for PDI programming and the following line to be defined:
#define MOSI_GATE 0 // PB0
#else
#error Unknown Arduino platform, help me define the correct pin layout
#endif

//
// Define to open a serial port for debugging
//
#undef DEBUG_ISP
#undef DEBUG_HVSP
#undef DEBUG_HVPP

#if defined(DEBUG_ISP) || defined(DEBUG_HVSP) || defined(DEBUG_HVPP)
#define SMO_WANT_DEBUG
#endif

#define DEFAULTTIMEOUT 100UL // default timeout anything (ms)

//
// Some pins used in multiple modules
//
enum {
    SMO_HVRESET    = 10,
#if SMO_LAYOUT==SMO_LAYOUT_STANDARD
    SMO_SVCC       = A0,
    SMO_DEBUG      = A5
#elif SMO_LAYOUT==SMO_LAYOUT_LEONARDO
    SMO_SVCC       = 11,
    SMO_DEBUG      = A4
#elif SMO_LAYOUT==SMO_LAYOUT_MEGA
    SMO_SVCC       = 11,
    SMO_DEBUG      = 18
#elif SMO_LAYOUT==SMO_LAYOUT_HVPROG2
    SMO_HVENABLE   = 11,
#define SMO_AVCC     31
#define FIVEVOLT     203   // 255 * 5.00 V / 6.29V
#define THREEVOLT    185   // a little more than 255 * 3.30 V / 5.00V
    SMO_SVCC       = 13,
    SMO_VADJ       = 12,   // for original STK500 board
    SMO_DEBUG      = 24,
    SMO_GLED       = 29,
    SMO_RLED       = 28
#endif
};

#endif /* _SMO_CONFIG_ */

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
