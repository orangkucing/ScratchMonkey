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

#define POINTER_UNCHANGED          0
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
// PDI has a functionality so called "Drive Contention and Collision Detection," and the target chip
// very really easily regards there is a collision on the data line. To avoid this, simple 
// impedance-matching mixture of MISO and MOSI doesn't always work, and thus an external tri-state
// buffer might be required: We must explicitly switch the direction of PDI_DATA line using it.
//
// Here we assume the tri-state buffer is 74HC125 or similar and its gate is connected to PDI_DATA_GATE pin.

#if SMO_LAYOUT==SMO_LAYOUT_STANDARD
// use SS pin to open/close gate of the tri-state buffer 74HC125
inline void
MOSI_ACTIVE(void)
{
    PORTB &= ~_BV(2); // PB2 = SS;
}

inline void
MOSI_TRISTATE(void)
{
    PORTB |= _BV(2); // PB2 = SS;
}

inline void
MOSI_GATE_INIT(void)
{
    PORTB |= _BV(2); // gate closed
    DDRB |= _BV(2); // PB2 = SS;
}

#elif SMO_LAYOUT==SMO_LAYOUT_LEONARDO
// use D10 pin to open/close gate of the tri-state buffer 74HC125
inline void
MOSI_ACTIVE(void)
{
    PORTB &= ~_BV(6); // PB6 = D10;
}

inline void
MOSI_TRISTATE(void)
{
    PORTB |= _BV(6); // PB6 = D10;
}

inline void
MOSI_GATE_INIT(void)
{
    PORTB |= _BV(6); // gate closed
    DDRB |= _BV(6); // PB6 = D10;
}

#elif SMO_LAYOUT==SMO_LAYOUT_MEGA
// use SS pin to open/close gate of the tri-state buffer 74HC125
inline void
MOSI_ACTIVE(void)
{
    PORTB &= ~_BV(0); // PB0 = SS;
}

inline void
MOSI_TRISTATE(void)
{
    PORTB |= _BV(0); // PB0 = SS;
}

inline void
MOSI_GATE_INIT(void)
{
    PORTB |= _BV(0); // gate closed
    DDRB |= _BV(0); // PB0 = SS;
}

#elif SMO_LAYOUT==SMO_LAYOUT_HVPROG2
// use PROGRST pin to open/close gate of the tri-state buffer 74HC125
inline void
MOSI_ACTIVE(void)
{
    PORTD |= _BV(2);
}

inline void
MOSI_TRISTATE(void)
{
    PORTD &= ~_BV(2);
}

inline void
MOSI_GATE_INIT(void)
{
    PORTD &= ~_BV(2); // gate closed   
    DDRD |= _BV(2);
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
    PORTB |= _BV(3) | _BV(4); // PB3 = MOSI; PB4 = MISO;
    SPCR &= ~_BV(SPE);        // temporary disable SPI
    PORTB &= ~_BV(5);         // PB5 = SCK;
    PORTB |= _BV(5);          // PB5 = SCK;
    SPCR |= _BV(SPE);         // enable SPI again
#elif SMO_LAYOUT==SMO_LAYOUT_LEONARDO || SMO_LAYOUT==SMO_LAYOUT_MEGA
    PORTB |= _BV(2) | _BV(3); // PB2 = MOSI; PB3 = MISO;
    SPCR &= ~_BV(SPE);        // temporary disable SPI
    PORTB &= ~_BV(1);         // PB1 = SCK;
    PORTB |= _BV(1);          // PB1 = SCK;
    SPCR |= _BV(SPE);         // enable SPI again
#elif SMO_LAYOUT==SMO_LAYOUT_HVPROG2
    PORTB |= _BV(5) | _BV(6); // PB5 = MOSI; PB6 = MISO;
    SPCR &= ~_BV(SPE);        // temporary disable SPI
    PORTB &= ~_BV(7);         // PB7 = SCK;
    PORTB |= _BV(7);          // PB7 = SCK;
    SPCR |= _BV(SPE);         // enable SPI again
#endif
}

inline void
HeartBeatOn(void)
{
    // Set timer operation mode and prescaler 1/8
#if SMO_LAYOUT==SMO_LAYOUT_HVPROG2
#if defined(TCCR2)
    TCCR2  = _BV(WGM21) | _BV(CS21);
#else
    TCCR2B = _BV(CS21);
#endif
#else
    TCCR1B = _BV(CS11);
#endif
}

inline void
HeartBeatOff(void)
{
#if SMO_LAYOUT==SMO_LAYOUT_HVPROG2
#if defined(TCCR2)
    TCCR2  = 0;
#else
    TCCR2B = 0;
#endif
#else
    TCCR1B = 0;
#endif
}

static void
EnableHeartBeat(void)
{    
    SPI.transfer(0xFF);
    SPI.transfer(0xFF);

    // to keep the established PDI connection active we must send dummy pulse periodically
    noInterrupts();
    // start timer
    // interrupts are generated in every 36us (7.3728MHz HVprog2) or 32us (16MHz Arduinos)
#if F_CPU <= 8000000
#define CMV 32
#elif F_CPU <= 16000000
#define CMV 64
#else
#define CMV 80
#endif
#if SMO_LAYOUT==SMO_LAYOUT_HVPROG2
#if defined(TCCR2)
    TCCR2  = _BV(WGM21);               // Stop Timer 2
    TCNT2  = 0xFF;                     // Initialize counter value
    OCR2   = CMV;                      // Set compare match value
    // CTC mode. Set timer operation mode and prescaler 1/8
    TIMSK  |= _BV(OCIE2);              // TIMER2_COMP interrupt enabled
    TCCR2  = _BV(WGM21) | _BV(CS21);
#else
    TCCR2B = 0;                        // Stop Timer 2
    TCCR2A = _BV(WGM21);               // CTC mode
    TCNT2  = 0xFF;                     // Initialize counter value
    OCR2A  = CMV;                      // Set compare match value
    TIMSK2 |= _BV(OCIE2A);             // TIMER2_COMPA interrupt enabled
    TCCR2B = _BV(CS21);                // Set timer operation mode and prescaler 1/8
#endif
#else
    TCCR1B = 0;                        // Stop clock generator
    TCCR1A = _BV(WGM12);               // CTC mode
    TCNT1  = 0xFF;                     // Initialize counter value
    OCR1A  = CMV;                      // Set compare match value
    TIMSK1 |= _BV(OCIE1A);             // TIMER1_COMPA interrupt enabled
    TCCR1B = _BV(CS11);                // Set timer operation mode and Prescaler 1/8
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

inline uint8_t
Parity(uint8_t data)
{
    // compute partiy bit
    uint8_t par = data;
    par ^= (par >> 4); // b[7:4] xor b[3:0]
    par ^= (par >> 2); // b[3:2] xor b[1:0]
    par ^= (par >> 1); // b[1:1] xor b[0:0]
    return par & 1;
}

/*
* send two byte in one PDI frame (24 bits)
* (1 start + 8 data + 1 parity + 2 stop) * 2
* using 3 SPI data bytes (3 x 8 = 24 clocks)
*/
static void
PDITransfer(uint8_t c, uint8_t d)
{
    uint8_t buf[3];
    // REMEMBER: this is in LSBfirst mode and idle is high
    // (1 start bit) + (c[6:0])
    buf[0] = c << 1;
    // (c[7:7]) + (1 parity) + (2 stop bits) + (1 start bit) + (d[2:0])
    buf[1] = (c >> 7) | (Parity(c) << 1) | B00001100 | (d << 5);
    // (d[7:3]) + (1 parity) + (2 stop bits)
    buf[2] = (d >> 3) | (Parity(d) << 5) | B11000000;
    HeartBeatOff();
    SPI.transfer(buf, 3);
    HeartBeatOn();
}

// send three bytes
static void
PDITransfer(uint8_t c, uint8_t d, uint8_t e)
{
    uint8_t buf[5];
    // (4 idle bits) + (1 start bit) + (data0[2:0])
    buf[0] = B00001111 | (c << 5);
    // (data0[7:3]) + (1 parity) + (2 stop bits)
    buf[1] = (c >> 3) | (Parity(c) << 5) | B11000000;
    // (1 start bit) + (d[6:0])
    buf[2] = d << 1;
    // (d[7:7]) + (1 parity) + (2 stop bits) + (1 start bit) + (e[2:0])
    buf[3] = (d >> 7) | (Parity(d) << 1) | B00001100 | (e << 5);
    // (d[7:3]) + (1 parity) + (2 stop bits)
    buf[4] = (e >> 3) | (Parity(e) << 5) | B11000000;
    HeartBeatOff();
    SPI.transfer(buf, 5);
    HeartBeatOn();
}

/*
* receive PDI 12-bit format byte data
*/
static void
PDITransfer(uint8_t c, uint8_t *p, uint8_t n=0)
{
    union {
      uint32_t i;
      uint8_t c[4];
    } data;
    uint8_t b;
    uint8_t buf[2];
    // (4 idle bits) + (1 start bit) + (c[2:0])
    buf[0] = B00001111 | (c << 5);
    // (c[7:3]) + (1 parity) + (2 stop bits)
    buf[1] = (c >> 3) | (Parity(c) << 5) | B11000000;
    HeartBeatOff();
    SPI.transfer(buf, 2);
    data.c[0] = 0xFF;
    MOSI_TRISTATE();
    do {
        while (data.c[0] == 0xFF) { // wait for the start bit
            SPDR = 0xFF;
            asm volatile("nop"); while (!(SPSR & _BV(SPIF))) ; // wait
            data.c[0] = SPDR;
        }
        SPDR = 0xFF;
        asm volatile("nop"); while (!(SPSR & _BV(SPIF))) ; // wait
        data.c[1] = SPDR;
        // now the received 8 bit data is contained in data.i
        b = 0;
        while (data.c[0] != 0x7F) {
            b++;
            data.i <<= 1;
            data.i |= 1;
        }
        *p++ = data.c[1];
        if (b < 4) { // a part of parity bit and stop bits is included in next byte
            data.i >>= b;
            SPDR = 0xFF;
            asm volatile("nop"); while (!(SPSR & _BV(SPIF))) ; // wait
            data.c[2] = SPDR; // this also contains partial next 8 bit data
            data.i <<= b;
            data.c[2] |= B00000111;
            data.i >>= b;
            data.c[0] = data.c[2];
        } else {
            data.c[2] |= B00000111; // overwrite parity bit and stop bits to 1
            data.i >>= b;
            data.c[0] = data.c[1];
        }
    } while (n--);
    MOSI_ACTIVE();
    HeartBeatOn();
}

static void
PDITransfer(uint8_t c, uint32_t a)
{
    // 5 = 3 + 2
    PDITransfer(c, (uint8_t)(a & 0xFF), (uint8_t)((a >> 8) & 0xFF));
    PDITransfer((uint8_t)((a >> 16) & 0xFF), (uint8_t)((a >> 24) & 0xFF));
}

static void
PDITransfer(uint8_t c, uint32_t a, uint8_t d)
{
    // 6 = 2 + 2 + 2
    PDITransfer(c, (uint8_t)(a & 0xFF));
    PDITransfer((uint8_t)((a >> 8) & 0xFF), (uint8_t)((a >> 16) & 0xFF));
    PDITransfer((uint8_t)((a >> 24) & 0xFF), d);
}

static void
PDISendKey(void)
{
    // 1 + 8 = 3 + 2 + 2 + 2
    PDITransfer((uint8_t)PDI_CMD_KEY, (uint8_t)0xFF, (uint8_t)0x88);
    PDITransfer((uint8_t)0xD8, (uint8_t)0xCD);
    PDITransfer((uint8_t)0x45, (uint8_t)0xAB);
    PDITransfer((uint8_t)0x89, (uint8_t)0x12);
}

static void
PDIEnableTarget(void)
{
#if SMO_LAYOUT==SMO_LAYOUT_HVPROG2
    // make sure that 12V regulator is shutdown 
    digitalWrite(SMO_HVENABLE, LOW);
    pinMode(SMO_HVENABLE, OUTPUT);
#endif
    MOSI_GATE_INIT(); // GATE closed
    digitalWrite(MISO, LOW);
    pinMode(MISO, OUTPUT);
    // release RESET/PDI_CLK line pulled-up by target
    pinMode(SCK, INPUT);
    delay(10);
    digitalWrite(MOSI, HIGH);
    pinMode(MOSI, OUTPUT);
    MOSI_ACTIVE(); // gate open
    digitalWrite(MISO, HIGH); // MISO line is untouched until the first SPI packet
    // to avoid glitch in SCK and MOSI we must set CPOL before SPI.begin()
    SPCR |= _BV(CPOL) | _BV(CPHA); // idle HIGH for SPI_MODE3
    //
    SPI.begin();
    SPI.beginTransaction(SPISettings(2000000, LSBFIRST, SPI_MODE3)); // set as fast as possible...
    delayMicroseconds(50);

    EnableHeartBeat();
}

static void
PDIDisableTarget(void)
{
    DisableHeartBeat();

    SPI.endTransaction();
    SPI.end();
}

static void
WaitUntilNVMActive(void)
{
    /* Wait until the bus between PDI controller and NVM becomes active */
    uint8_t s;
    do {
        PDITransfer(PDI_CMD_LDCS | REG_STATUS, &s);
    } while (!(s & _BV(NVMEN)));
}

static void
WaitWhileNVMControllerBusy(void)
{
    uint8_t s;
    PDITransfer(PDI_CMD_ST | POINTER_REG | Data32_t, SMoXPROG::XPRGParam.NVMBase | NVMREG_STATUS);
    do {
        PDITransfer(PDI_CMD_LD | POINTER_UNCHANGED | Data8_t, &s);
    } while (s & _BV(NVMBSY));
}

//----------------------------------------------------------------------------------------

uint16_t
SMoPDI::EnterProgmode()
{
    /* Enable PDI programming mode with the attached target */
    PDIEnableTarget();

    /* Store the RESET key into the RESET PDI registor */
    PDITransfer(PDI_CMD_STCS | REG_RESET, (uint8_t)0x59); // reset key
    
    /* Direction change guard time to 0 USART bits */
    //PDITransfer(PDI_CMD_STCS | REG_CTRL, (uint8_t)(GT2 | GT1 | GT0));

    /* Enable access to the XPROG NVM bus by sending the documented NVM access key to the device */
    PDISendKey();
 
    WaitUntilNVMActive();

    XPRG_Body[1] = XPRG_ERR_OK;
    return 2;
}

uint16_t
SMoPDI::LeaveProgmode()
{
    uint8_t s;

    WaitWhileNVMControllerBusy();
    do {
        PDITransfer(PDI_CMD_STCS | REG_RESET, (uint8_t)0x00);
        PDITransfer(PDI_CMD_LDCS | REG_RESET, &s);
    } while (s);

    PDIDisableTarget();

    XPRG_Body[1] = XPRG_ERR_OK;
    return 2;
}

uint16_t
SMoPDI::Erase()
{
    uint8_t mode     = XPRG_Body[1];
    SMoGeneral::gAddress.c[3] = XPRG_Body[2];
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
    case XPRG_ERASE_APP_PAGE:
        command = ERASE_APP_PAGE;
        break;
    case XPRG_ERASE_BOOT_PAGE:
        command = ERASE_BOOT_PAGE;
        break;
    case XPRG_ERASE_EEPROM_PAGE:
        command = ERASE_EEPROM_PAGE;
        break;
    case XPRG_ERASE_USERSIG:
        command = ERASE_USER_SIG_ROW;
        break;
    default:
        XPRG_Body[1] = XPRG_ERR_FAILED;
        return 2;
    break;
    }

    PDITransfer(PDI_CMD_ST | POINTER_REG | Data32_t, SMoXPROG::XPRGParam.NVMBase | NVMREG_CMD);
    PDITransfer(PDI_CMD_ST | POINTER_UNCHANGED | Data8_t, command);
    if (command == CHIP_ERASE || command == ERASE_EEPROM) {
        PDITransfer(PDI_CMD_ST | POINTER_REG | Data8_t, (uint8_t)NVMREG_CTRLA);
        PDITransfer(PDI_CMD_ST | POINTER_UNCHANGED | Data8_t, (uint8_t)_BV(CMDEX));
    } else {
        PDITransfer(PDI_CMD_ST | POINTER_REG | Data32_t, SMoGeneral::gAddress.l);
        PDITransfer(PDI_CMD_ST | POINTER_UNCHANGED | Data8_t, (uint8_t)0xFF); // dummy write
    }
    if (command == CHIP_ERASE)
        WaitUntilNVMActive();
    WaitWhileNVMControllerBusy();
    
    XPRG_Body[1] = XPRG_ERR_OK;
    return 2;
}

uint16_t
SMoPDI::WriteMem()
{
    uint8_t memcode = XPRG_Body[1];
    uint8_t mode = XPRG_Body[2];
    SMoGeneral::gAddress.c[3] = XPRG_Body[3];
    SMoGeneral::gAddress.c[2] = XPRG_Body[4];
    SMoGeneral::gAddress.c[1] = XPRG_Body[5];
    SMoGeneral::gAddress.c[0] = XPRG_Body[6];
    uint16_t numBytes = (XPRG_Body[7]<<8)|XPRG_Body[8];
    uint8_t *data = &XPRG_Body[9];
    
    uint8_t command = LOAD_FLASH_BUFFER;
    uint8_t erase = ERASE_FLASH_BUFFER;
    uint8_t command1;
    bool paged = true;
    switch (memcode) {
    case XPRG_MEM_TYPE_APPL:
        command1 = mode & _BV(XPRG_MEM_WRITE_ERASE) ? ERASE_WRITE_APP_PAGE : WRITE_APP_PAGE;
        break;
    case XPRG_MEM_TYPE_BOOT:
        command1 = mode & _BV(XPRG_MEM_WRITE_ERASE) ? ERASE_WRITE_BOOT_PAGE : WRITE_BOOT_PAGE;
        break;
    case XPRG_MEM_TYPE_EEPROM:
        command = LOAD_EEPROM_BUFFER;
        erase = ERASE_EEPROM_BUFFER;
        command1 = mode & _BV(XPRG_MEM_WRITE_ERASE) ? ERASE_WRITE_EEPROM_PAGE : WRITE_EEPROM_PAGE;
        break;
    case XPRG_MEM_TYPE_FUSE:
        command = WRITE_FUSES;
        paged = false;
        break;
    case XPRG_MEM_TYPE_LOCKBITS:
        command = WRITE_LOCK_BITS;
        paged = false;
        break;
    case XPRG_MEM_TYPE_USERSIG:
        command1 = WRITE_USER_SIG_ROW;  
        break;
    default:
        XPRG_Body[1] = XPRG_ERR_FAILED;
        return 2;
    }

    if (paged) {
        if (numBytes & 1)
            data[numBytes++] = 0xFF; // always transfer even bytes
        // load page buffer
        PDITransfer(PDI_CMD_STS | Addr32_t | Data8_t, SMoXPROG::XPRGParam.NVMBase | NVMREG_CMD, command);
        PDITransfer(PDI_CMD_ST | POINTER_REG | Data32_t, SMoGeneral::gAddress.l);
        PDITransfer(PDI_CMD_REPEAT | Data8_t, (uint8_t)(numBytes - 1), (uint8_t)(PDI_CMD_ST | POINTER_POST_INC | Data8_t));
        do {
            PDITransfer(*data, *(data+1));
            data += 2;
            numBytes -= 2;
        } while (numBytes);
        if (mode & _BV(XPRG_MEM_WRITE_WRITE)) {
            WaitWhileNVMControllerBusy();
            // write page
            PDITransfer(PDI_CMD_STS | Addr32_t | Data8_t, SMoXPROG::XPRGParam.NVMBase | NVMREG_CMD, command1);
            PDITransfer(PDI_CMD_STS | Addr32_t | Data8_t, SMoGeneral::gAddress.l, (uint8_t)0xFF); // dummy write
            WaitWhileNVMControllerBusy();
        }
    } else // Byte mode
        PDITransfer(PDI_CMD_STS | Addr32_t | Data8_t, SMoXPROG::XPRGParam.NVMBase | NVMREG_CMD, command);
        PDITransfer(PDI_CMD_ST | POINTER_REG | Data32_t, SMoGeneral::gAddress.l);            
        do {
            PDITransfer(PDI_CMD_ST | POINTER_POST_INC | Data8_t, *data++);
            WaitWhileNVMControllerBusy();
        } while (--numBytes);

    XPRG_Body[1] = XPRG_ERR_OK;
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
    uint16_t numBytes = (XPRG_Body[6]<<8)|XPRG_Body[7];
    uint8_t *outData = &XPRG_Body[2];

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

    PDITransfer(PDI_CMD_STS | Addr32_t | Data8_t, SMoXPROG::XPRGParam.NVMBase | NVMREG_CMD, command);
    PDITransfer(PDI_CMD_ST | POINTER_REG | Data32_t, SMoGeneral::gAddress.l);
    PDITransfer(PDI_CMD_REPEAT | Data8_t, (uint8_t)(numBytes - 1));
    PDITransfer(PDI_CMD_LD | POINTER_POST_INC | Data8_t, outData, (uint8_t)(numBytes - 1));

    XPRG_Body[1] = XPRG_ERR_OK;
    return numBytes + 2;
}

uint16_t
SMoPDI::CRC()
{
    uint8_t type = XPRG_Body[1];
    uint8_t *outData = &XPRG_Body[2];
    
    uint8_t command;
    switch (type) {
    case XPRG_CRC_APP:
        command = APP_CRC;
        break;
    case XPRG_CRC_BOOT:
        command = BOOT_CRC;
        break;
    case XPRG_CRC_FLASH:
        command = FLASH_CRC;
        break;
    default:
        XPRG_Body[1] = XPRG_ERR_FAILED;
        return 2;
    }
 
    PDITransfer(PDI_CMD_ST | POINTER_REG | Data32_t, SMoXPROG::XPRGParam.NVMBase | NVMREG_CMD);
    PDITransfer(PDI_CMD_ST | POINTER_UNCHANGED | Data8_t, (uint8_t)command);
    PDITransfer(PDI_CMD_ST | POINTER_REG | Data8_t, (uint8_t)NVMREG_CTRLA);
    PDITransfer(PDI_CMD_ST | POINTER_UNCHANGED | Data8_t, (uint8_t)_BV(CMDEX));
    if (command == FLASH_CRC)
        WaitUntilNVMActive();
    WaitWhileNVMControllerBusy();
    PDITransfer(PDI_CMD_ST | POINTER_REG | Data8_t, (uint8_t)NVMREG_DATA0);
    PDITransfer(PDI_CMD_LD | POINTER_POST_INC | Data8_t, outData++);
    PDITransfer(PDI_CMD_LD | POINTER_POST_INC | Data8_t, outData++);
    PDITransfer(PDI_CMD_LD | POINTER_UNCHANGED | Data8_t, outData);

    uint8_t b = *outData;
    *outData = *(outData-2);
    *(outData-2) = b;
    
    XPRG_Body[1] = XPRG_ERR_OK;
    return 5;
}
