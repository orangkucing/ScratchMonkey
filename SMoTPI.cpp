// -*- mode: c++; tab-width: 4; indent-tabs-mode: nil -*-
//
// ScratchMonkey 0.1        - STK500v2 compatible programmer for Arduino
//
// File: SMoTPI.cpp       - Tiny Programming Interface (TPI) 
//

// This part of ScratchMonkey is written by Hisashi Ito <info at mewpro.cc> (c) 2015
// in order to support HVprog2, an STK500 clone open hardware that you can buy or make.
// http://www.mewpro.cc
//
// Derived from source codes of LUFA AVRISP mkII clone, avrdude,
// and ATtiny4_5_9_10_20_40Programmer.ino

// The same license as main part applies.

#include <SPI.h>
#include "SMoCommand.h"
#include "SMoGeneral.h"
#include "SMoConfig.h"
#include "SMoXPROG.h"
#include "SMoTPI.h"

//
// Pin definitions
//
enum {
    TPI_VCC         = SMO_SVCC,        // toggling VCC is optional unless RSTDISBL fuse is programmed
    TPI_HVRESET     = SMO_HVRESET,     // HV reset is also optional.
#if SMO_LAYOUT==SMO_LAYOUT_STANDARD
    TPI_RESET       = SS,
#elif SMO_LAYOUT==SMO_LAYOUT_LEONARDO
    TPI_RESET       = 10,
#elif SMO_LAYOUT==SMO_LAYOUT_MEGA
    TPI_RESET       = SS,
#elif SMO_LAYOUT==SMO_LAYOUT_HVPROG2
    TPI_RESET       = SS,
#endif
};

#define TIMEOUT 100

//                                 B0x10pi00 x=rw, p=direct, i=postinc
#define TPI_CMD_SLD                B00100000
#define TPI_CMD_SST                B01100000
#define TPI_CMD_SSTPR              B01101000
//                                 Bxaa1aaaa x=rw, aaaaaa=address(+NVMBASE)
#define TPI_CMD_SIN(a)            (B00010000 | (a & B110000) << 1 | (a & B1111))
#define TPI_CMD_SOUT(a)           (B10010000 | (a & B110000) << 1 | (a & B1111))
//                                 B1x00rrrr x=rw, rrrr=Control/Status register                                     
#define TPI_CMD_SLDCS              B10000000
#define TPI_CMD_SSTCS              B11000000
//
#define TPI_CMD_SKEY               B11100000

#define REG_TPISR                  0x00
#define NVMEN    1

#define REG_TPIPCR                 0x02
#define GT0      0
#define GT1      1
#define GT2      2

#define REG_TPIIR                  0x0F

#define POINTER_UNCHANGED          0
#define POINTER_POST_INC           B00000100

//
// NVM Programming commands
#define NO_OPERATION           0x00
#define CHIP_ERASE             0x10
#define SECTION_ERASE          0x14
#define WORD_WRITE             0x1D // ATtiny4/5/9/10
#define DWORD_WRITE            WORD_WRITE // ATtiny20
#define CODE_WRITE             WORD_WRITE // ATtiny40
// Non-Volatile Memory Busy
#define NVMBSY 7

static int8_t sPageMask;

inline uint8_t
Parity(uint8_t d)
{
    // compute partiy bit
    uint8_t par = d;
    par ^= (par >> 4); // b[7:4] xor b[3:0]
    par ^= (par >> 2); // b[3:2] xor b[1:0]
    par ^= (par >> 1); // b[1:1] xor b[0:0]
    return par & 1;
}

/*
* send two byte in one TPI frame (24 bits)
* (1 start + 8 data + 1 parity + 2 stop) * 2
* using 3 SPI data bytes (3 x 8 = 24 clocks)
*/
static void
TPITransfer(uint8_t c, uint8_t d)
{
    // REMEMBER: this is in LSBfirst mode and idle is high
    // (1 start bit) + (c[6:0])
    SPI.transfer(c << 1);
    // (c[7:7]) + (1 parity) + (2 stop bits) + (1 start bit) + (d[2:0])
    SPI.transfer((c >> 7) | (Parity(c) << 1) | B00001100 | (d << 5));
    // (d[7:3]) + (1 parity) + (2 stop bits)
    SPI.transfer((d >> 3) | (Parity(d) << 5) | B11000000);
}

/*
* receive TPI 12-bit format byte data
* via SPI 2 bytes (16 clocks) or 3 bytes (24 clocks)
*/
static void
TPITransfer(uint8_t c, uint8_t *p)
{
    union {
      uint16_t i;
      uint8_t c[2];
    } data;
    SPI.transfer(0xFF); // idle
    // (4 idle bits) + (1 start bit) + (c[2:0])
    SPI.transfer(B00001111 | (c << 5));
    // (c[7:3]) + (1 parity) + (2 stop bits)
    SPI.transfer((c >> 3) | (Parity(c) << 5) | B11000000);

    do { // wait for the start bit
        data.c[0] = SPI.transfer(0xFF);
    } while (data.c[0] == 0xFF);
    data.c[1] = SPI.transfer(0xFF);
    // if the first byte data.c[0] contains less than 3 data bits
    // we need to get a third byte to get the parity and/or stop bits.
    if ((data.c[0] & B00011111) == B00011111)
        SPI.transfer(0xFF);
    // now the received 8 bit data is contained in data.i
    while (data.c[0] != 0x7F) {
        data.i <<= 1;
        data.i |= 1;
    }
    SPI.transfer(0xFF);
    SPI.transfer(0xFF);
    *p = data.c[1];
}

static void
TPISendKey(void)
{
    // (4 idle bits) + (1 start bit) + (data0[2:0])
    SPI.transfer(B00001111 | (TPI_CMD_SKEY << 5));
    // (data0[7:3]) + (1 parity) + (2 stop bits)
    SPI.transfer((TPI_CMD_SKEY >> 3) | (Parity(TPI_CMD_SKEY) << 5) | B11000000);

    TPITransfer((uint8_t)0xFF, (uint8_t)0x88); // keys
    TPITransfer((uint8_t)0xD8, (uint8_t)0xCD);
    TPITransfer((uint8_t)0x45, (uint8_t)0xAB);
    TPITransfer((uint8_t)0x89, (uint8_t)0x12);
}

static void
TPISendIdle(void)
{
    // send equal to or more than 12 idle bits
    SPI.transfer(0xFF);
    SPI.transfer(0xFF);
}

static void
TPIEnableTarget(void)
{
    // enter TPI programming mode
    
    // power off target (optional)
    digitalWrite(TPI_VCC, LOW);
    pinMode(TPI_VCC, OUTPUT);
    delayMicroseconds(250);
    // and target RESET = LOW, HVRESET=0V
    digitalWrite(TPI_RESET, LOW);
    pinMode(TPI_RESET, OUTPUT);
    digitalWrite(TPI_HVRESET, HIGH);
    pinMode(TPI_HVRESET, OUTPUT);
#if SMO_LAYOUT==SMO_LAYOUT_HVPROG2
    digitalWrite(SMO_HVENABLE, HIGH);
    pinMode(SMO_HVENABLE, OUTPUT); // enable 12V
#endif

    // make sure power is off completely
    {
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
    digitalWrite(TPI_VCC, HIGH);
    delayMicroseconds(250);
    // target RESET = minimum 400ns positive pulse
    // target HVRESET = 12V
    digitalWrite(TPI_HVRESET, LOW);
    digitalWrite(TPI_RESET, HIGH); // rising edge of the positive pulse
    
    // set control pins
    // to avoid glitch in SCK and MOSI we must set CPOL before SPI.begin()
    SPCR |= _BV(CPOL) | _BV(CPHA); // idle HIGH for SPI_MODE3
    //
    SPI.begin();
    SPI.beginTransaction(SPISettings(2000000, LSBFIRST, SPI_MODE3)); // max clock freq for programming is 2MHz

    digitalWrite(TPI_RESET, LOW); // falling edge of the positive pulse
    // delay
    delayMicroseconds(SMoGeneral::gDischargeDelay);
    SPI.transfer(0xFF); // activate TPI by emitting 16 or more pulses on TPICLK
    SPI.transfer(0xFF);
}

static void
TPIDisableTarget(void)
{
    SPI.endTransaction();
    SPI.end();
    // deassert reset
    digitalWrite(TPI_RESET, HIGH);
#if SMO_LAYOUT==SMO_LAYOUT_HVPROG2
    digitalWrite(SMO_HVENABLE, LOW); // disable 12V
#endif
}

static bool
WaitWhileNVMControllerBusy(void)
{
    uint8_t s;
    uint8_t timeout = TIMEOUT;
    do {
        TPITransfer(TPI_CMD_SIN(SMoXPROG::XPRGParam.NVMCSR), &s);
        if (timeout-- == 0)
            return false;
    } while (s & _BV(NVMBSY));
    return true;
}

//----------------------------------------------------------------------------------------

uint16_t
SMoTPI::EnterProgmode()
{
    /* Enable TPI programming mode with the attached target */
    TPIEnableTarget();

    /* Direction change guard time to 0 CLK bits */
    TPITransfer(TPI_CMD_SSTCS | REG_TPIPCR, (uint8_t)(GT2 | GT1 | GT0));

    /* Enable access to the XPROG NVM bus by sending the documented NVM access key to the device */
    TPISendKey();

    /* Wait until the NVM bus becomes active */
    uint8_t s;
    do {
        /* Send the SLDCS command to read the TPI STATUS register to see the NVM bus is active */
        TPITransfer(TPI_CMD_SLDCS | REG_TPISR, &s);
    } while (!(s & _BV(NVMEN)));

 //-----------------------------------------------------------------------------------------------   
    /* determine write size */
    TPITransfer(TPI_CMD_SOUT(SMoXPROG::XPRGParam.NVMCMD), (uint8_t)NO_OPERATION);
    TPITransfer(TPI_CMD_SSTPR | 0, 0xC1); // location of the second byte of signature
    TPITransfer(TPI_CMD_SSTPR | 1, 0x3F);

    uint8_t c;
    TPITransfer(TPI_CMD_SLD | POINTER_UNCHANGED, (uint8_t *)&c);
    if (c < 0x90) // 512B Flash memory
        sPageMask = 1; // 2 bytes at a time
    else // 2^(c & 0x0F) KB Flash memory
        sPageMask = _BV((c & 0x0F) + 1) - 1; // 2^((c & 0x0F) + 1) bytes at a time 
//-----------------------------------------------------------------------------------------------    
    XPRG_Body[1] = XPRG_ERR_OK;
    return 2;
}

uint16_t
SMoTPI::LeaveProgmode()
{
    uint8_t s;
    do {
        TPITransfer(TPI_CMD_SSTCS | REG_TPISR, (uint8_t)0x00);
        TPITransfer(TPI_CMD_SLDCS | REG_TPISR, &s);
    } while (s);

    TPIDisableTarget();
    XPRG_Body[1] = XPRG_ERR_OK;
    return 2;
}

uint16_t
SMoTPI::Erase()
{
    uint8_t mode     = XPRG_Body[1];
    //SMoGeneral::gAddress.c[3] = XPRG_Body[2];
    //SMoGeneral::gAddress.c[2] = XPRG_Body[3];
    //SMoGeneral::gAddress.c[1] = XPRG_Body[4];
    //SMoGeneral::gAddress.c[0] = XPRG_Body[5];
  
    TPITransfer(TPI_CMD_SOUT(SMoXPROG::XPRGParam.NVMCMD), mode == XPRG_ERASE_CHIP ? (uint8_t)CHIP_ERASE : (uint8_t)SECTION_ERASE);

    TPITransfer(TPI_CMD_SSTPR | 0, XPRG_Body[5] | 1);
    TPITransfer(TPI_CMD_SSTPR | 1, XPRG_Body[4]);
    TPITransfer(TPI_CMD_SST | POINTER_UNCHANGED, (uint8_t)0xFF);

    XPRG_Body[1] = WaitWhileNVMControllerBusy() ? XPRG_ERR_OK : XPRG_ERR_TIMEOUT;
    return 2;
}

uint16_t
SMoTPI::WriteMem()
{
    uint8_t memcode = XPRG_Body[1];
    //uint8_t mode = XPRG_Body[2];
    //SMoGeneral::gAddress.c[3] = XPRG_Body[3];
    //SMoGeneral::gAddress.c[2] = XPRG_Body[4];
    //SMoGeneral::gAddress.c[1] = XPRG_Body[5];
    //SMoGeneral::gAddress.c[0] = XPRG_Body[6];
    uint16_t numBytes = (XPRG_Body[7]<<8)|XPRG_Body[8];
    uint8_t *data = &XPRG_Body[9];

    int8_t writeSize = memcode == XPRG_MEM_TYPE_LOCKBITS ? 1 : sPageMask;

    TPITransfer(TPI_CMD_SOUT(SMoXPROG::XPRGParam.NVMCMD), (uint8_t)WORD_WRITE);
    if (numBytes & 1)
        data[numBytes++] = 0xFF; // padding

    /* Send the address of the location to write to */
    TPITransfer(TPI_CMD_SSTPR | 0, XPRG_Body[6]);
    TPITransfer(TPI_CMD_SSTPR | 1, XPRG_Body[5]);

    uint16_t i = 0;
    do {
        do {
            TPITransfer(TPI_CMD_SST | POINTER_POST_INC, *data++);
            i++;
            TPITransfer(TPI_CMD_SST | POINTER_POST_INC, *data++);
            i++;
            if (i == numBytes) {
                while (i & sPageMask) { // write dummy bytes
                    TPISendIdle();
                    TPITransfer(TPI_CMD_SST | POINTER_POST_INC, (uint8_t)0xFF);
                    i++;
                    TPITransfer(TPI_CMD_SST | POINTER_POST_INC, (uint8_t)0xFF);
                    i++;
                }
                break;
            }
            if ((i & sPageMask) == 0)
                break;
            TPISendIdle();
        } while (1);
        if (!WaitWhileNVMControllerBusy()) {
            XPRG_Body[1] = XPRG_ERR_TIMEOUT;
            return 2;
        }        
    } while (i < numBytes);

    XPRG_Body[1] = XPRG_ERR_OK;
    return 2;
}

uint16_t
SMoTPI::ReadMem()
{
    //uint8_t memcode = XPRG_Body[1];
    //SMoGeneral::gAddress.c[3] = XPRG_Body[2];
    //SMoGeneral::gAddress.c[2] = XPRG_Body[3];
    //SMoGeneral::gAddress.c[1] = XPRG_Body[4];
    //SMoGeneral::gAddress.c[0] = XPRG_Body[5];
    uint16_t numBytes = (XPRG_Body[6]<<8)|XPRG_Body[7];
    uint8_t *outData = &XPRG_Body[2];
    
    /* Set the NVM control register to the NO OP command for memory reading */
    TPITransfer(TPI_CMD_SOUT(SMoXPROG::XPRGParam.NVMCMD), (uint8_t)NO_OPERATION);

    /* Send the address of the location to read from */
    TPITransfer(TPI_CMD_SSTPR | 0, XPRG_Body[5]);
    TPITransfer(TPI_CMD_SSTPR | 1, XPRG_Body[4]);

    for (uint16_t i = 0; i < numBytes; i++) {
        TPITransfer(TPI_CMD_SLD | POINTER_POST_INC, outData++);
    }
    XPRG_Body[1] = XPRG_ERR_OK;
    return numBytes + 2;
}

uint16_t
SMoTPI::CRC()
{
    XPRG_Body[1] = XPRG_ERR_FAILED;
    return 5;
}
