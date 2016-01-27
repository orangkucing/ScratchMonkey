// -*- mode: c++; tab-width: 4; indent-tabs-mode: nil -*-
//
// ScratchMonkey 0.1        - STK500v2 compatible programmer for Arduino
//
// File: SMoTPI.cpp       - Tiny Programming Interface (TPI) 
//

// This part of ScratchMonkey is written by Hisashi Ito <info at mewpro.cc> (c) 2015
// in order to support HVprog2, an STK500 clone open hardware that you can buy or make.
// http://mewpro.cc/2016/01/20/how-to-use-hvprog2/
//
// Derived from source codes of LUFA AVRISP mkII clone, avrdude,
// and ATtiny4_5_9_10_20_40Programmer.ino

// The same license as main part applies.

#include <SPI.h>
#include <util/parity.h>
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
#if SMO_LAYOUT==SMO_LAYOUT_STANDARD
    TPI_HVRESET     = 9,               // HV reset is optional.
    TPI_RESET       = 8,
#elif SMO_LAYOUT==SMO_LAYOUT_LEONARDO
    TPI_HVRESET     = 9,               // HV reset is optional.
    TPI_RESET       = 8,
#elif SMO_LAYOUT==SMO_LAYOUT_MEGA
    TPI_HVRESET     = 9,               // HV reset is optional.
    TPI_RESET       = 8,
#elif SMO_LAYOUT==SMO_LAYOUT_HVPROG2
    TPI_HVRESET     = SMO_HVRESET,     // HV reset is optional.
    TPI_RESET       = SS,
#endif
};

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

/*
* send two byte in two TPI frame (24 bits)
* (1 start + 8 data + 1 parity + 2 stop) * 2
* using 3 SPI data bytes (3 x 8 = 24 clocks)
*/
static void
TPIStore(uint8_t c, uint8_t d)
{
    union {
      uint32_t l;
      uint8_t c[4];
    } data;

    data.l = 0UL;
    data.c[1] = d;
    data.l <<= 4;
    data.c[0] = c;
    data.l <<= 1;
    data.c[1] |= parity_even_bit(c) << 1 | 0x0C;
    data.c[2] |= parity_even_bit(d) << 5 | 0xC0;
    SPI.transfer((uint8_t*)&data.c[0], 3);
}

/*
* send one byte in one TPI frame (12 bits)
* 4 idle + 1 start + 8 data + 1 parity + 2 stop
* using 2 SPI data bytes (2 x 8 = 16 clocks)
*/
static void
TPITransfer(uint8_t c)
{
    union {
      uint16_t i;
      uint8_t c[2];
    } data;
    
    data.i = 0U;
    data.c[0] = c;
    data.i <<= 5;
    data.i |= parity_even_bit(c) << 13 | 0xC00F;
    SPI.transfer((uint8_t*)&data.c[0], 2);
}

/*
* receive TPI 12-bit format byte data
* via SPI 3 bytes (24 clocks)
*/
static void
TPILoad(uint8_t c, uint8_t *p)
{
    union {
      uint32_t l;
      uint8_t c[4];
    } data;

    TPITransfer(c);
    do { // wait for the start bit
        data.c[1] = SPI.transfer(0xFF);
    } while (data.c[1] == 0xFF);
    data.c[0] = 0xFF; // sentinel
    data.c[2] = SPI.transfer(0xFF);
    // now the received 8 bit data is contained in data.c[2:1]
    // Note: If the position of the start bit is within bit [7:5] of data.c[1]
    // then 1 - 3 bits of the parity bit and/or stop bits are not received, yet.
    SPI.transfer(0xFF); // receive remaining bits if any
    while (data.c[1] != 0x7F)
        data.l <<= 1;
    *p = data.c[2];
}

static void
TPISendKey(void)
{
    TPITransfer(TPI_CMD_SKEY);
    TPIStore(0xFF, 0x88); // keys
    TPIStore(0xD8, 0xCD);
    TPIStore(0x45, 0xAB);
    TPIStore(0x89, 0x12);
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
#ifdef SMO_AVCC
    analogWrite(TPI_VCC, 0);
#else
    digitalWrite(TPI_VCC, LOW);
    pinMode(TPI_VCC, OUTPUT);
#endif
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
#ifdef SMO_AVCC
    uint32_t time = millis();
    while (analogRead(SMO_AVCC) > 10) { // wait until TPI_VCC becomes lower than 0.06V
        if (millis() - time > DEFAULTTIMEOUT)
            break;
    }
#else
    delay(DEFAULTTIMEOUT);
#endif
    
    // power on target
#ifdef SMO_AVCC
    analogWrite(TPI_VCC, 255);
    {
        uint32_t time = millis();
        while (analogRead(SMO_AVCC) < 743) {   // wait until TPI_VCC becomes higher than 4.5V
            if (millis() - time > DEFAULTTIMEOUT)
                break;
        }
    }
    analogWrite(TPI_VCC, FIVEVOLT);
#else
    digitalWrite(TPI_VCC, HIGH);
#endif
    // target RESET = minimum 400ns positive pulse
    // target HVRESET = 12V
    digitalWrite(TPI_HVRESET, LOW);
    digitalWrite(TPI_RESET, HIGH); // rising edge of the positive pulse
    
    // set control pins
    pinMode(MOSI, OUTPUT);    // specify the data direction of MOSI and SCK to become SPI master
    pinMode(SCK, OUTPUT);
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
    uint32_t time = millis();
    do {
        TPILoad(TPI_CMD_SIN(SMoXPROG::XPRGParam.NVMCSR), &s);
        if (millis() - time > DEFAULTTIMEOUT)
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
    TPIStore(TPI_CMD_SSTCS | REG_TPIPCR, GT2 | GT1 | GT0);

    /* Enable access to the XPROG NVM bus by sending the documented NVM access key to the device */
    TPISendKey();

    /* Wait until the NVM bus becomes active */
    uint8_t s;
    do {
        /* Send the SLDCS command to read the TPI STATUS register to see the NVM bus is active */
        TPILoad(TPI_CMD_SLDCS | REG_TPISR, &s);
    } while (!(s & _BV(NVMEN)));

 //-----------------------------------------------------------------------------------------------   
    /* determine write size */
    TPIStore(TPI_CMD_SOUT(SMoXPROG::XPRGParam.NVMCMD), NO_OPERATION);
    TPIStore(TPI_CMD_SSTPR | 0, 0xC1); // location of the second byte of signature
    TPIStore(TPI_CMD_SSTPR | 1, 0x3F);

    uint8_t c;
    TPILoad(TPI_CMD_SLD | POINTER_UNCHANGED, &c);
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
    TPIStore(TPI_CMD_SSTCS | REG_TPISR, 0x00);
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
  
    TPIStore(TPI_CMD_SOUT(SMoXPROG::XPRGParam.NVMCMD), mode == XPRG_ERASE_CHIP ? CHIP_ERASE : SECTION_ERASE);

    TPIStore(TPI_CMD_SSTPR | 0, XPRG_Body[5] | 1);
    TPIStore(TPI_CMD_SSTPR | 1, XPRG_Body[4]);
    TPIStore(TPI_CMD_SST | POINTER_UNCHANGED, 0x55); // dummy write

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
    uint16_t numBytes = XPRG_Body[7] << 8 | XPRG_Body[8];
    uint8_t *data = &XPRG_Body[9];

    int8_t writeSize = memcode == XPRG_MEM_TYPE_LOCKBITS ? 1 : sPageMask;

    TPIStore(TPI_CMD_SOUT(SMoXPROG::XPRGParam.NVMCMD), WORD_WRITE);
    if (numBytes & 1)
        data[numBytes++] = 0xFF; // padding

    /* Send the address of the location to write to */
    TPIStore(TPI_CMD_SSTPR | 0, XPRG_Body[6]);
    TPIStore(TPI_CMD_SSTPR | 1, XPRG_Body[5]);

    uint16_t i = 0;
    do {
        do {
            TPIStore(TPI_CMD_SST | POINTER_POST_INC, *data++);
            i++;
            TPIStore(TPI_CMD_SST | POINTER_POST_INC, *data++);
            i++;
            if (i == numBytes) {
                while (i & sPageMask) { // write dummy bytes
                    TPISendIdle();
                    TPIStore(TPI_CMD_SST | POINTER_POST_INC, 0xFF);
                    i++;
                    TPIStore(TPI_CMD_SST | POINTER_POST_INC, 0xFF);
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
    uint16_t numBytes = XPRG_Body[6] << 8 | XPRG_Body[7];
    uint8_t *outData = &XPRG_Body[2];
    
    /* Set the NVM control register to the NO OP command for memory reading */
    TPIStore(TPI_CMD_SOUT(SMoXPROG::XPRGParam.NVMCMD), NO_OPERATION);

    /* Send the address of the location to read from */
    TPIStore(TPI_CMD_SSTPR | 0, XPRG_Body[5]);
    TPIStore(TPI_CMD_SSTPR | 1, XPRG_Body[4]);

    for (uint16_t i = 0; i < numBytes; i++) {
        TPILoad(TPI_CMD_SLD | POINTER_POST_INC, outData++);
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
