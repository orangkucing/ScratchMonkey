// -*- mode: c++; tab-width: 4; indent-tabs-mode: nil -*-
//
// ScratchMonkey 0.1        - STK500v2 compatible programmer for Arduino
//
// File: SMoPDI.cpp       - PDI Programming Interface
//

// This part of ScratchMonkey is written by Hisashi Ito <info at mewpro.cc> (c) 2015
// in order to support HVprog2, an STK500 clone open hardware that you can buy or make.
// http://mewpro.cc/2016/01/20/how-to-use-hvprog2/
//
// Derived from source codes of LUFA AVRISP mkII clone, avrdude, USBasp
// and ATtiny4_5_9_10_20_40Programmer.ino

// The same license as main part applies.

#ifndef _SMO_PDI_
#define _SMO_PDI_

namespace SMoPDI {
    extern uint16_t EnterProgmode();
    extern uint16_t LeaveProgmode();
    extern uint16_t Erase();
    extern uint16_t WriteMem();
    extern uint16_t ReadMem();
    extern uint16_t CRC();
} // namespace SMoPDI

#endif /* _SMO_PDI_ */
