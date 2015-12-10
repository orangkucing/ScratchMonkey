// -*- mode: c++; tab-width: 4; indent-tabs-mode: nil -*-
//
// ScratchMonkey 0.1        - STK500v2 compatible programmer for Arduino
//
// File: SMoPDI.cpp       - PDI Programming Interface
//

// This part of ScratchMonkey is written by Hisashi Ito <info at mewpro.cc> (c) 2015
// in order to support HVprog2, an STK500 clone open hardware that you can buy or make.
// http://www.mewpro.cc
//
// Derived from source codes of LUFA AVRISP mkII clone, avrdude.

// The same license as main part applies.

#include <SPI.h>
#include <util/parity.h>
#include "SMoCommand.h"
#include "SMoGeneral.h"
#include "SMoConfig.h"
#include "SMoXPROG.h"
#include "SMoPDI.h"

//                                 B0x10pidd x=rw, p=direct, i=postinc, dd=datasize
#define PDI_CMD_LD                 B00100000
#define PDI_CMD_ST                 B01100000
//                                 B0x00aadd x=rw, aa=addresssize, dd=datasize
#define PDI_CMD_LDS                B00000000
#define PDI_CMD_STS                B01000000
//                                 B1x00rrrr x=rw, rrrr=Control/Status register
#define PDI_CMD_LDCS               B10000000
#define PDI_CMD_STCS               B11000000
//
#define PDI_CMD_KEY                B11100000
//                                 B101000dd dd=datasize
#define PDI_CMD_REPEAT             B10100000

#define REG_STATUS                 0x00
#define NVMEN    1

#define REG_RESET                  0x01

#define REG_CTRL                   0x02
#define GT0      0
#define GT1      1
#define GT2      2

#define POINTER_UNCHANGED          B00000000
#define POINTER_POST_INC           B00000100
#define POINTER_REG                B00001000

#define Data8_t                    B00000000
#define Data16_t                   B00000001
#define Data24_t                   B00000010
#define Data32_t                   B00000011
#define Addr8_t                    B00000000
#define Addr16_t                   B00000100
#define Addr24_t                   B00001000
#define Addr32_t                   B00001100
//
// NVM commands available for external programming
#define NO_OPERATION               0x00
#define CHIP_ERASE                 0x40
#define READ_NVM                   0x43
// Flash Page Buffer
#define LOAD_FLASH_BUFFER          0x23
#define ERASE_FLASH_BUFFER         0x26
// Flash
#define ERASE_FLASH_PAGE           0x2B
#define WRITE_FLASH_PAGE           0x2E
#define ERASE_WRITE_FLASH_PAGE     0x2F
#define FLASH_CRC                  0x78
// Application Section
#define ERASE_APP                  0x20
#define ERASE_APP_PAGE             0x22
#define WRITE_APP_PAGE             0x24
#define ERASE_WRITE_APP_PAGE       0x25
#define APP_CRC                    0x38
// Boot Loader Seciton
#define ERASE_BOOT                 0x68
#define ERASE_BOOT_PAGE            0x2A
#define WRITE_BOOT_PAGE            0x2C
#define ERASE_WRITE_BOOT_PAGE      0x2D
#define BOOT_CRC                   0x39
// Production Signature (Calibration) and User Signature Row
#define READ_USER_SIG_ROW          0x01
#define ERASE_USER_SIG_ROW         0x18
#define WRITE_USER_SIG_ROW         0x1A
#define READ_CALIB_ROW             0x02
// Fuse and Lock Bit Commands
#define READ_FUSES                 0x07
#define WRITE_FUSES                0x4C
#define WRITE_LOCK_BITS            0x08
// EEPROM Page Buffer
#define LOAD_EEPROM_BUFFER         0x33
#define ERASE_EEPROM_BUFFER        0x36
// EEPROM
#define ERASE_EEPROM               0x30
#define ERASE_EEPROM_PAGE          0x32
#define WRITE_EEPROM_PAGE          0x34
#define ERASE_WRITE_EEPROM_PAGE    0x35
#define READ_EEPROM                0x06

// Offset from SMoXPROG::XPRGParam.NVMBase
#define NVMREG_ADDR0               0x00
#define NVMREG_ADDR1               0x01
#define NVMREG_ADDR2               0x02
#define NVMREG_DATA0               0x04
#define NVMREG_DATA1               0x05
#define NVMREG_DATA2               0x06
#define NVMREG_CMD                 0x0A
#define NVMREG_CTRLA               0x0B
#   define CMDEX                      0
#define NVMREG_CTRLB               0x0C
#define NVMREG_INTCTRL             0x0D
#define NVMREG_STATUS              0x0F
#   define NVMBSY 7
#define NVMREG_LOCKBITS            0x10

//
// Pin definitions
//

#if SMO_LAYOUT==SMO_LAYOUT_STANDARD
inline void
MOSI_ACTIVE(void)
{
    DDRB |= _BV(3); // PB3 = MOSI;
}

inline void
MOSI_TRISTATE(void)
{
    DDRB &= ~_BV(3); // PB3 = MOSI;
}
#elif SMO_LAYOUT==SMO_LAYOUT_LEONARDO || SMO_LAYOUT==SMO_LAYOUT_MEGA
inline void
MOSI_ACTIVE(void)
{
    DDRB |= _BV(2); // PB2 = MOSI;
}

inline void
MOSI_TRISTATE(void)
{
    DDRB &= ~_BV(2); // PB2 = MOSI;
}
#elif SMO_LAYOUT==SMO_LAYOUT_HVPROG2
inline void
MOSI_ACTIVE(void)
{
    DDRB |= _BV(5); // PB5 = MOSI;
}

inline void
MOSI_TRISTATE(void)
{
    DDRB &= ~_BV(5); // PB5 = MOSI;
}
#endif

#if SMO_LAYOUT==SMO_LAYOUT_HVPROG2
#if defined(TIMER2_COMP_vect)
#define TIMER_COMP_vect TIMER2_COMP_vect
#else
#define TIMER_COMP_vect TIMER2_COMPA_vect
#endif
#else
#define TIMER_COMP_vect TIMER1_COMPA_vect
#endif

ISR(TIMER_COMP_vect)
{
#if SMO_LAYOUT==SMO_LAYOUT_STANDARD
    PORTB ^= _BV(5);          // PB5 = SCK;
#elif SMO_LAYOUT==SMO_LAYOUT_LEONARDO || SMO_LAYOUT==SMO_LAYOUT_MEGA
    PORTB ^= _BV(1);          // PB1 = SCK;
#elif SMO_LAYOUT==SMO_LAYOUT_HVPROG2
    PORTB ^= _BV(7) ;         // PB7 = SCK;
#endif
}

inline void
HeartBeatOn(void)
{
    noInterrupts();
    // Set timer operation mode and prescaler 1/8
#if SMO_LAYOUT==SMO_LAYOUT_HVPROG2
#if defined(TCCR2)
    TCCR2  = _BV(WGM21) | _BV(CS21);
#else
    TCCR2B = _BV(WGM22) | _BV(CS21);
#endif
#else
    TCCR1B = _BV(WGM12) | _BV(CS11);
#endif
   SPCR &= ~_BV(SPE);         // temporary disable SPI
   interrupts();
}

inline void
HeartBeatOff(void)
{
    noInterrupts();
#if SMO_LAYOUT==SMO_LAYOUT_HVPROG2
#if defined(TCCR2)
    TCCR2  = 0;
#else
    TCCR2B = 0;
#endif
#else
    TCCR1B = 0;
#endif
    SPCR |= _BV(SPE);         // enable SPI again
    interrupts();
}

static void
EnableHeartBeat(void)
{
    // set PDI_DATA high for < 100us to disable RESET function on PDI_CLK
    // Note: Since RESET line in a circuit might have capacitance longer delay is better 
    delayMicroseconds(58);
    SPI.transfer(0xFF); // send 16 clock pulses to PDI_CLK
    SPI.transfer(0xFF);

    // to keep the established PDI connection active we must send dummy pulse periodically
    noInterrupts();
    // start timer
    // interrupts are generated in every 18us (7.3728MHz HVprog2) or 16us (16MHz Arduinos)
#if F_CPU <= 8000000
#define CMV 16
#elif F_CPU <= 16000000
#define CMV 32
#else
#define CMV 40
#endif
#if SMO_LAYOUT==SMO_LAYOUT_HVPROG2
#if defined(TCCR2)
    TCCR2  = 0;                        // Stop Timer 2
    TCNT2  = 0;                        // Initialize counter value
    OCR2   = CMV;                      // Set compare match value
    // CTC mode. Set timer operation mode and prescaler 1/8
    TIMSK  |= _BV(OCIE2);              // TIMER2_COMP interrupt enabled
    TCCR2  = _BV(WGM21) | _BV(CS21);
#else
    TCCR2B = 0;                        // Stop Timer 2
    TCCR2A = 0;                        // CTC mode
    TCNT2  = 0;                        // Initialize counter value
    OCR2A  = CMV;                      // Set compare match value
    TIMSK2 |= _BV(OCIE2A);             // TIMER2_COMPA interrupt enabled
    TCCR2B = _BV(WGM22) | _BV(CS21);   // Set timer operation mode and prescaler 1/8
#endif
#else
    TCCR1B = 0;                        // Stop clock generator
    TCCR1A = 0;                        // CTC mode
    TCNT1  = 0;                        // Initialize counter value
    OCR1A  = CMV;                      // Set compare match value
    TIMSK1 |= _BV(OCIE1A);             // TIMER1_COMPA interrupt enabled
    TCCR1B = _BV(WGM12) | _BV(CS11);   // Set timer operation mode and Prescaler 1/8
#endif
    interrupts();
}

static void
DisableHeartBeat(void)
{
    // stop timer
    noInterrupts();
#if SMO_LAYOUT==SMO_LAYOUT_HVPROG2
#if defined(TCCR2)
    TCCR2  = 0;
    TIMSK &= ~_BV(OCIE2);
#else
    TCCR2B = 0;
    TCCR2A = 0;
    TIMSK2 &= ~_BV(OCIE2A);
#endif
#else
    TIMSK1 &= ~_BV(OCIE1A);
    TCCR1B = 0;
#endif
    interrupts();
}

/*
* send two byte in two PDI frame (24 bits)
* (1 start + 8 data + 1 parity + 2 stop) * 2
* using 3 SPI data bytes (3 x 8 = 24 clocks)
*/
static void
PDIStore(uint8_t c, uint8_t d)
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
    HeartBeatOff();
    SPI.transfer((uint8_t*)&data.c[0], 3);
    HeartBeatOn();
}

/*
* send one byte in one PDI frame (12 bits)
* 4 idle + 1 start + 8 data + 1 parity + 2 stop
* using 2 SPI data bytes (2 x 8 = 16 clocks)
*/
static void
PDITransfer(uint8_t d, bool cont=false)
{
    union {
      uint16_t i;
      uint8_t c[2];
    } data;

    data.i = 0U;
    data.c[0] = d;
    data.i <<= 5;
    data.i |= parity_even_bit(d) << 13 | 0xC00F;
    HeartBeatOff();
    SPI.transfer((uint8_t*)&data.c[0], 2);
    if (!cont)
        HeartBeatOn();
}

/*
* receive PDI 12-bit format byte data
*/
static void
PDILoad(uint8_t c, uint8_t *p, uint8_t n)
{
    union {
      uint32_t l;
      uint8_t c[4];
    } data;
    uint8_t b;
 
    PDITransfer(c, true);
    MOSI_TRISTATE();
    data.c[1] = SPI.transfer(0xFF);
    do {
        while (data.c[1] == 0xFF) // wait for the start bit
            data.c[1] = SPI.transfer(0xFF);
        b = 0;
        data.c[0] = 0xFF; // sentinel
        data.c[2] = SPI.transfer(0xFF);
        // now the received 8 bit data is contained in data.l
        while (data.c[1] != 0x7F) {
            data.l <<= 1;
            b++;
        }
        *p++ = data.c[2];
        if (b > 3) {
            data.c[2] = 0xFF;
            data.c[3] |= B00000111; // overwrite parity bit and stop bits to 1
            data.l >>= 8 + b;           
        } else {
            data.c[1] = SPI.transfer(0xFF) | B00000111 >> b;
        }
    } while (n--);
    MOSI_ACTIVE();
    HeartBeatOn();
}

static void
PDILoadS(uint8_t c, uint32_t a, uint8_t *p)
{
    PDIStore(c, a & 0xFF);
    PDIStore((a >> 8) & 0xFF, (a >> 16) & 0xFF);
    PDILoad((a >> 24) & 0xFF, p, 0);
}

static void
PDIStoreS(uint8_t c, uint32_t a)
{
    PDITransfer(c);
    PDIStore(a & 0xFF, (a >> 8) & 0xFF);
    PDIStore((a >> 16) & 0xFF, (a >> 24) & 0xFF);
}

static void
PDISendKey(void)
{
    PDITransfer(PDI_CMD_KEY);
    PDIStore(0xFF, 0x88);
    PDIStore(0xD8, 0xCD);
    PDIStore(0x45, 0xAB);
    PDIStore(0x89, 0x12);
}

static void
PDIEnableTarget(void)
{
    pinMode(MISO, INPUT);
    digitalWrite(MOSI, LOW);
    pinMode(MOSI, OUTPUT);    // specify the data direction of MOSI and SCK to become SPI master
    pinMode(SCK, LOW);
    pinMode(SCK, OUTPUT);
    delay(100);               // set PDI_DATA low for a while
    digitalWrite(MOSI, HIGH); // to keep MOSI HIGH from HeartBeatOn() to HeartBeatOff()    
    SPDR = 0xFF;              // to keep MOSI HIGH before the first data (dummy clocks) is shifted out
    SPI.begin();
    SPI.beginTransaction(SPISettings(20000000, LSBFIRST, SPI_MODE3));
    EnableHeartBeat();
}

static void
PDIDisableTarget(void)
{
    DisableHeartBeat();
    SPI.endTransaction();
    SPI.end();
}

static bool
WaitUntilNVMActive(void)
{
    /* Wait until the bus between PDI controller and NVM becomes active */
    uint8_t s;
    uint32_t time = millis();
    do {
        if (millis() - time > DEFAULTTIMEOUT)
            return false;
        PDILoad(PDI_CMD_LDCS | REG_STATUS, &s, 0);
    } while (!(s & _BV(NVMEN)));
    return true;
}

static bool
WaitWhileNVMControllerBusy(void)
{
    uint8_t s;
    uint32_t time = millis();
    do {
        if (millis() - time > DEFAULTTIMEOUT)
            return false;
        // in order to preserve the PDI pointer register we use direct addressing
        PDILoadS(PDI_CMD_LDS | Addr32_t | Data8_t, SMoXPROG::XPRGParam.NVMBase + NVMREG_STATUS, &s);
    } while (s & _BV(NVMBSY));
    return true;
}

//----------------------------------------------------------------------------------------

uint16_t
SMoPDI::EnterProgmode()
{
    uint8_t s;
    
    PDIEnableTarget();
    /* Direction change guard time to 2 CLK bits */
    //PDIStore(PDI_CMD_STCS | REG_CTRL, _BV(GT2) | _BV(GT1) | _BV(GT0));
    PDIStore(PDI_CMD_STCS | REG_RESET, 0x59); // reset key
    PDISendKey();
    if (!WaitUntilNVMActive())
        goto EnterProgmodeERROR;
    XPRG_Body[1] = XPRG_ERR_OK;
    return 2;
    
EnterProgmodeERROR:
    XPRG_Body[1] = XPRG_ERR_TIMEOUT;
    return 2;
}

uint16_t
SMoPDI::LeaveProgmode()
{
    PDIStore(PDI_CMD_STCS | REG_RESET, 0);
    PDIStore(PDI_CMD_STCS | REG_STATUS, _BV(NVMEN));
    PDIDisableTarget();
    XPRG_Body[1] = XPRG_ERR_OK;
    return 2;    
}

uint16_t
SMoPDI::Erase()
{
    uint8_t mode     = XPRG_Body[1];
    SMoGeneral::gAddress.c[3] = XPRG_Body[2]; // the address parameter is ignored if erase mode is XPRG_ERASE_CHIP
    SMoGeneral::gAddress.c[2] = XPRG_Body[3];
    SMoGeneral::gAddress.c[1] = XPRG_Body[4];
    SMoGeneral::gAddress.c[0] = XPRG_Body[5];

    uint8_t command;
    switch (mode) {
    case XPRG_ERASE_CHIP:
        command = CHIP_ERASE;
        break;
    case XPRG_ERASE_APP:
        command = ERASE_APP;
        break;
    case XPRG_ERASE_BOOT:
        command = ERASE_BOOT;
        break;
    case XPRG_ERASE_EEPROM:
        command = ERASE_EEPROM;
        break;
    case XPRG_ERASE_USERSIG:
        command = ERASE_USER_SIG_ROW;
        break;
    /* no XPRG mode for the following command (non-functional for XMEGA-A1)
        command = ERASE_FLASH_PAGE;
        break; */
    case XPRG_ERASE_APP_PAGE:
        command = ERASE_APP_PAGE;
        break;
    case XPRG_ERASE_BOOT_PAGE:
        command = ERASE_BOOT_PAGE;
        break;
    case XPRG_ERASE_EEPROM_PAGE:
        command = ERASE_EEPROM_PAGE;
        break;
    default:
        XPRG_Body[1] = XPRG_ERR_FAILED;
        return 2;
    break;
    }
 
    PDIStoreS(PDI_CMD_STS | Addr32_t | Data8_t, SMoXPROG::XPRGParam.NVMBase + NVMREG_CMD); PDITransfer(command);
    if (command == CHIP_ERASE) {
        PDIStoreS(PDI_CMD_STS | Addr32_t | Data8_t, SMoXPROG::XPRGParam.NVMBase + NVMREG_CTRLA); PDITransfer(_BV(CMDEX));
    } else {
        PDIStoreS(PDI_CMD_STS | Addr32_t | Data8_t, SMoGeneral::gAddress.l); PDITransfer(0x55); // dummy write
    }
    if (command == CHIP_ERASE) {
        if (!WaitUntilNVMActive()) {
            XPRG_Body[1] = XPRG_ERR_TIMEOUT;
            return 2;
        }
    }
    XPRG_Body[1] = WaitWhileNVMControllerBusy() ? XPRG_ERR_OK : XPRG_ERR_TIMEOUT;
    return 2;
}

/* There are many typos in Atmel's application note AVR079, dated back to Apr. 2008. 
The following should be the correct statements written there about XPRG_WRITE_MEM:
----------------------------------------------------------------------------------------------------------
9.2.5 XPRG_WRITE_MEM 
This command handles programming of the different XMega memories: application, 
boot and eeprom. Fuses, lockbits and user signatures are also programmed with this 
command. 

Table 9-13. Command format. 
Offset Field                                       Size       Values 
0      Command ID (1)                              1 byte     XPRG_WRITE_MEM
1      Memory type (2)                             1 byte     Application, Boot, EEPROM, Fuse, Lockbits... 
2      PageMode (3)                                1 byte     Bitfield, see description below 
3      Address (4)                                 4 bytes    Any address 
7      Length (5)                                  2 bytes    1 to 256
9      Data (6)                                    N bytes    N data bytes, size is given by the Length parameter 

Notes:      1.    The command identifier 
            2.    XPRG_MEM_TYPE_APPL 
                  XPRG_MEM_TYPE_BOOT 
                  XPRG_MEM_TYPE_EEPROM 
                  XPRG_MEM_TYPE_FUSE 
                  XPRG_MEM_TYPE_LOCKBITS 
                  XPRG_MEM_TYPE_USERSIG 
            3.    If Memory type is XPRG_MEM_TYPE_APPL, XPRG_MEM_TYPE_BOOT or XPRG_MEM_TYPE_EEPROM: 
                  Bit 0: Erase page 
                  Bit 1: Write page 
            4.    The start address of the data to be written. The address is in the TIF address space 
            5.    Can be any value between 1 and 256. If page programming, and the actual page size is bigger than 256, the 
                  operation must be split into two or more XPRG_WRITE_MEM operations, where only the last operation has the 
                  Write page bit set. 
                  Note: Only APP, BOOT and EEPROM handles page operations, for any other memory type, the Length field must 
                  be set to 1. 
            4.    The data to be written. The size is indicated by the Length field. 
----------------------------------------------------------------------------------------------------------
Four years after this reference, Atmel Studio 5.1 (Feb. 2012) introduced a new parameter XPRG_PARAM_FLASHPAGESIZE to
STK600's XPROG protocol. The improvement is because the old implimentation imposed the PC side software on sending
data chunks of the same length as target's page size.

Here we are going to code so that both old and new methods will work. Note the PageMode bitfield included in any
new packet has the write page bit set.
 */
uint16_t
SMoPDI::WriteMem()
{
    uint8_t memcode = XPRG_Body[1];
    uint8_t mode = XPRG_Body[2];
    SMoGeneral::gAddress.c[3] = XPRG_Body[3];
    SMoGeneral::gAddress.c[2] = XPRG_Body[4];
    SMoGeneral::gAddress.c[1] = XPRG_Body[5];
    SMoGeneral::gAddress.c[0] = XPRG_Body[6];
    uint16_t numBytes = XPRG_Body[7] << 8 | XPRG_Body[8];
    uint8_t *data = &XPRG_Body[9];
    
    uint8_t command = LOAD_FLASH_BUFFER;
    uint8_t erase = ERASE_FLASH_BUFFER;
    uint8_t commit;
    uint16_t pageMask = 0;
    switch (memcode) {
    /* no XPRG memcode for the following command (non-functional for XMEGA-A1)
        commit = mode & _BV(XPRG_MEM_WRITE_ERASE) ? ERASE_WRITE_APP_FLASH : WRITE_APP_FLASH;
        pageMask = SMoXPROG::XPRGParam.FlashPageSize - 1;
        break; */
    case XPRG_MEM_TYPE_APPL:
        commit = mode & _BV(XPRG_MEM_WRITE_ERASE) ? ERASE_WRITE_APP_PAGE : WRITE_APP_PAGE;
        pageMask = SMoXPROG::XPRGParam.FlashPageSize - 1;
        break;
    case XPRG_MEM_TYPE_BOOT:
        commit = mode & _BV(XPRG_MEM_WRITE_ERASE) ? ERASE_WRITE_BOOT_PAGE : WRITE_BOOT_PAGE;
        pageMask = SMoXPROG::XPRGParam.FlashPageSize - 1;
        break;
    case XPRG_MEM_TYPE_EEPROM:
        command = LOAD_EEPROM_BUFFER;
        erase = ERASE_EEPROM_BUFFER;
        commit = mode & _BV(XPRG_MEM_WRITE_ERASE) ? ERASE_WRITE_EEPROM_PAGE : WRITE_EEPROM_PAGE;
        pageMask = SMoXPROG::XPRGParam.EEPageSize - 1;
        break;
    case XPRG_MEM_TYPE_FUSE:
        command = WRITE_FUSES;
        break;
    case XPRG_MEM_TYPE_LOCKBITS:
        command = WRITE_LOCK_BITS;
        break;
    case XPRG_MEM_TYPE_USERSIG:
        command = WRITE_USER_SIG_ROW;
        break;
    default:
        XPRG_Body[1] = XPRG_ERR_FAILED;
        return 2;
    }

    if (pageMask) {
        uint32_t startAddress = SMoGeneral::gAddress.l;
        uint8_t sendMask = pageMask > 0xFF ? 0xFF : pageMask;
        if (numBytes & 1)
            data[numBytes++] = 0xFF; // always transfer even bytes
        //
        // load page buffer
        //
        // Note: We don't need to erase page buffer since it is automatically erased after:
        //     1. A device reset
        //     2. Executing the WRITE_XXXX_PAGE or ERASE_WRITE_XXXX_PAGE command
        //     3. Executing the WRITE_USER_SIG_ROW or WRITE_LOCK_BITS command
        //
        PDIStoreS(PDI_CMD_STS | Addr32_t | Data8_t, SMoXPROG::XPRGParam.NVMBase + NVMREG_CMD); PDITransfer(command);
        PDIStoreS(PDI_CMD_ST | POINTER_REG | Data32_t, SMoGeneral::gAddress.l);
        do {
            PDIStore(PDI_CMD_REPEAT | Data8_t, sendMask - (uint8_t)(SMoGeneral::gAddress.l & sendMask));
            PDITransfer(PDI_CMD_ST | POINTER_POST_INC | Data8_t);
            do {
                if (numBytes) {
                    PDIStore(*data, *(data+1));
                    data += 2;
                    numBytes -= 2;
                } else {
                    PDIStore(0xFF, 0xFF);
                }
                SMoGeneral::gAddress.l += 2;
            } while (SMoGeneral::gAddress.l & sendMask);
            if (!WaitWhileNVMControllerBusy())
                goto WriteMemERROR;
            if ((numBytes == 0 || !(SMoGeneral::gAddress.l & pageMask)) && mode & _BV(XPRG_MEM_WRITE_WRITE)) {
                // write page
                PDIStoreS(PDI_CMD_STS | Addr32_t | Data8_t, SMoXPROG::XPRGParam.NVMBase + NVMREG_CMD); PDITransfer(commit);
                PDIStoreS(PDI_CMD_STS | Addr32_t | Data8_t, startAddress); PDITransfer(0x55); // dummy write
                if (!WaitWhileNVMControllerBusy())
                    goto WriteMemERROR;
            }
        } while (numBytes);
    } else { // Byte mode
        PDIStoreS(PDI_CMD_STS | Addr32_t | Data8_t, SMoXPROG::XPRGParam.NVMBase + NVMREG_CMD); PDITransfer(command);
        PDIStoreS(PDI_CMD_ST | POINTER_REG | Data32_t, SMoGeneral::gAddress.l);            
        do {
            PDIStore(PDI_CMD_ST | POINTER_POST_INC | Data8_t, *data++);
            if (!WaitWhileNVMControllerBusy())
                goto WriteMemERROR;
        } while (--numBytes);
    }    
    XPRG_Body[1] = XPRG_ERR_OK;
    return 2;

WriteMemERROR:
    XPRG_Body[2] = XPRG_ERR_TIMEOUT;
    return 2;
}

uint16_t
SMoPDI::ReadMem()
{
    uint8_t type = XPRG_Body[1];
    SMoGeneral::gAddress.c[3] = XPRG_Body[2];
    SMoGeneral::gAddress.c[2] = XPRG_Body[3];
    SMoGeneral::gAddress.c[1] = XPRG_Body[4];
    SMoGeneral::gAddress.c[0] = XPRG_Body[5];
    uint16_t numBytes = XPRG_Body[6] << 8 | XPRG_Body[7];

    uint8_t command;
    switch (type) {
    case XPRG_MEM_TYPE_APPL:
    case XPRG_MEM_TYPE_BOOT:
        command = READ_NVM;
        break;
    case XPRG_MEM_TYPE_EEPROM:
        command = READ_EEPROM;
        break;
    case XPRG_MEM_TYPE_FUSE:
    case XPRG_MEM_TYPE_LOCKBITS:
        command = READ_FUSES;
        break;
    case XPRG_MEM_TYPE_USERSIG:
        command = READ_USER_SIG_ROW;
        break;
    case XPRG_MEM_TYPE_FACTORY_CALIBRATION:
        command = READ_CALIB_ROW;
        break;
    default:
        XPRG_Body[1] = XPRG_ERR_FAILED;
        return 2;
    }

    PDIStoreS(PDI_CMD_STS | Addr32_t | Data8_t, SMoXPROG::XPRGParam.NVMBase + NVMREG_CMD); PDITransfer(command);
    PDIStoreS(PDI_CMD_ST | POINTER_REG | Data32_t, SMoGeneral::gAddress.l);
    if (numBytes > 1)
        PDIStore(PDI_CMD_REPEAT | Data8_t, numBytes - 1);
    PDILoad(PDI_CMD_LD | POINTER_POST_INC | Data8_t, &XPRG_Body[2], numBytes - 1);

    XPRG_Body[1] = XPRG_ERR_OK;
    return numBytes + 2;
}

uint16_t
SMoPDI::CRC()
{
    uint8_t type = XPRG_Body[1];
    uint8_t *outData = &XPRG_Body[2];
    
    uint8_t command, b;
    switch (type) {
    case XPRG_CRC_APP:
        command = APP_CRC;
        break;
    case XPRG_CRC_BOOT:
        command = BOOT_CRC;
        break;
    case XPRG_CRC_FLASH:
        command = FLASH_CRC; //  (non-functional for XMEGA-A1)
        break;
    default:
        XPRG_Body[1] = XPRG_ERR_FAILED;
        return 2;
    }

    PDIStoreS(PDI_CMD_STS | Addr32_t | Data8_t, SMoXPROG::XPRGParam.NVMBase + NVMREG_CMD); PDITransfer(command);
    PDIStoreS(PDI_CMD_STS | Addr32_t | Data8_t, SMoXPROG::XPRGParam.NVMBase + NVMREG_CTRLA); PDITransfer(_BV(CMDEX));
    if (command == FLASH_CRC && !WaitUntilNVMActive() || !WaitWhileNVMControllerBusy())
        goto CRCERROR;
    PDILoadS(PDI_CMD_LDS | Addr32_t | Data8_t, SMoXPROG::XPRGParam.NVMBase + NVMREG_DATA2, outData++);
    PDILoadS(PDI_CMD_LDS | Addr32_t | Data8_t, SMoXPROG::XPRGParam.NVMBase + NVMREG_DATA1, outData++);
    PDILoadS(PDI_CMD_LDS | Addr32_t | Data8_t, SMoXPROG::XPRGParam.NVMBase + NVMREG_DATA0, outData++);
    XPRG_Body[1] = XPRG_ERR_OK;
    return 5;

CRCERROR:
    XPRG_Body[1] = XPRG_ERR_TIMEOUT;
    return 2;
}
