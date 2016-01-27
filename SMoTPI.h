// -*- mode: c++; tab-width: 4; indent-tabs-mode: nil -*-
//
// ScratchMonkey 0.1        - STK500v2 compatible programmer for Arduino
//
// File: SMoTPI.h       - Tiny Programming Interface (TPI) 
//

// This part of ScratchMonkey is written by Hisashi Ito <info at mewpro.cc> (c) 2015
// in order to support HVprog2, an STK500 clone open hardware that you can buy or make.
// http://mewpro.cc/2016/01/20/how-to-use-hvprog2/
//
// Derived from source codes of LUFA AVRISP mkII clone, avrdude, USBasp
// and ATtiny4_5_9_10_20_40Programmer.ino

// The same license as main part applies.

// To use ScratchMonkey for (low voltage) TPI programming, connect the following pins
// of your Arduino (On the Leonardo, some of these are on the ICSP header):
//
// PIN          Standard        Leonardo/Micro  Mega (1280 and 2560)
//
// RESET            10             10              53 
//   *              11           MOSI              51                          * this pin is connected through 3.9k ohm register to TPIDATA pin 
// TPIDATA          12           MISO              50 
// TPICLK           13            SCK              52 
//
// If you are to do high voltage TPI programming then use
// RESET            10             10              10
// VCC              A0             11              11
// pins as RESET and VCC respectively.
//


#ifndef _SMO_TPI_
#define _SMO_TPI_

namespace SMoTPI {
    extern uint16_t EnterProgmode();
    extern uint16_t LeaveProgmode();
    extern uint16_t Erase();
    extern uint16_t WriteMem();
    extern uint16_t ReadMem();
    extern uint16_t CRC();
} // namespace SMoTPI

#endif /* _SMO_TPI_ */
