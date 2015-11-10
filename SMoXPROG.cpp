#include <SPI.h>
#include "SMoCommand.h"
#include "SMoGeneral.h"
#include "SMoConfig.h"
#include "SMoXPROG.h"

//
// Pin definitions
//
enum {
    TPI_VCC         = SMO_SVCC,        // toggling VCC is optional unless RSTDISBL fuse is programmed
#if SMO_LAYOUT==SMO_LAYOUT_STANDARD
    TPI_RESET       = SS,
#elif SMO_LAYOUT==SMO_LAYOUT_LEONARDO
    TPI_RESET       = 10,
#elif SMO_LAYOUT==SMO_LAYOUT_MEGA
    TPI_RESET       = SS,
#elif SMO_LAYOUT==SMO_LAYOUT_HVPROG2
    TPI_HVRESET     = SMO_HVRESET,
    TPI_RESET       = SS,
#endif
};

#define XPRG_Body (SMoCommand::gBody+1)

static struct {
    uint8_t NVMCMD = 0x33; // Non-Volatile Memory Command Register
// NVM Programming commands
#define NO_OPERATION           0x00
#define CHIP_ERASE             0x10
#define SECTION_ERASE          0x14
#define WORD_WRITE             0x1D // ATtiny4/5/9/10
#define DWORD_WRITE            WORD_WRITE // ATtiny20
#define CODE_WRITE             WORD_WRITE // ATtiny40

    uint8_t NVMCSR = 0x32; // Non-Volatile Memory Control and Status Register
// Non-Volatile Memory Busy
#define NVMBSY 7

} XPRGParam;

//
#define TPI_CMD_SLD                B00100000
#define TPI_CMD_SST                B01100000
#define TPI_CMD_SSTPR              B01101000
#define TPI_CMD_SIN(a)            (B00010000 | (a & B110000) << 1 | (a & B1111))
#define TPI_CMD_SOUT(a)           (B10010000 | (a & B110000) << 1 | (a & B1111))
#define TPI_CMD_SLDCS              B10000000
#define TPI_CMD_SSTCS              B11000000
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

static uint8_t SelectedProtocol = XPRG_MODE_TPI;

static int8_t sPageMask;
static uint16_t sAnswerLen;

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
* send two byte in one TPI frame (24 bits)
* (1 start + 8 data + 1 parity + 2 stop) * 2
* using 3 SPI data bytes (3 x 8 = 24 clocks)
*/
static void
XPRGTransfer(uint8_t data0, uint8_t data1)
{
    // REMEMBER: this is in LSBfirst mode and idle is high
    // (1 start bit) + (data0[6:0])
    SPI.transfer(data0 << 1);
    // (data0[7:7]) + (1 parity) + (2 stop bits) + (1 start bit) + (data1[2:0])
    SPI.transfer((data0 >> 7) | (Parity(data0) << 1) | B00001100 | (data1 << 5));
    // (data1[7:3]) + (1 parity) + (2 stop bits)
    SPI.transfer((data1 >> 3) | (Parity(data1) << 5) | B11000000);
}

/*
* receive TPI 12-bit format byte data
* via SPI 2 bytes (16 clocks) or 3 bytes (24 clocks)
*/
static void
XPRGTransfer(uint8_t c, uint8_t *p)
{
    union {
      uint16_t i;
      uint8_t c[2];
    } data;
    // (4 idle bits) + (1 start bit) + (data0[2:0])
    SPI.transfer(B00001111 | (c << 5));
    // (data0[7:3]) + (1 parity) + (2 stop bits)
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
    *p = data.c[1];
}

static void
XPRGSendKey(void)
{
    // (4 idle bits) + (1 start bit) + (data0[2:0])
    SPI.transfer(B00001111 | (TPI_CMD_SKEY << 5));
    // (data0[7:3]) + (1 parity) + (2 stop bits)
    SPI.transfer((TPI_CMD_SKEY >> 3) | (Parity(TPI_CMD_SKEY) << 5) | B11000000);

    XPRGTransfer((uint8_t)0xFF, (uint8_t)0x88); // keys
    XPRGTransfer((uint8_t)0xD8, (uint8_t)0xCD);
    XPRGTransfer((uint8_t)0x45, (uint8_t)0xAB);
    XPRGTransfer((uint8_t)0x89, (uint8_t)0x12);
}

static void
XPRGSendIdle(void)
{
    // send equal to or more than 12 idle bits
    SPI.transfer(0xFF);
    SPI.transfer(0xFF);
}

static void
XPRGEnableTargetTPI(void)
{
    // enter TPI programming mode
    // power off target (optional)
    digitalWrite(TPI_VCC, LOW);
    pinMode(TPI_VCC, OUTPUT);
    delayMicroseconds(250);
    // target RESET = LOW
    //digitalWrite(TPI_RESET, LOW);
    //pinMode(TPI_RESET, OUTPUT);
#if SMO_LAYOUT==SMO_LAYOUT_HVPROG2
    digitalWrite(TPI_HVRESET, HIGH);
    pinMode(TPI_HVRESET, OUTPUT);
    digitalWrite(SMO_HVENABLE, HIGH);
    pinMode(SMO_HVENABLE, OUTPUT); // enable 12V
#endif
    delay(100);
    // power on target
    digitalWrite(TPI_VCC, HIGH);
    delayMicroseconds(250);
    // target RESET = minimum 400ns positive pulse
#if SMO_LAYOUT==SMO_LAYOUT_HVPROG2
    // target HVRESET = 12V
    digitalWrite(TPI_HVRESET, LOW);
#endif
    //digitalWrite(TPI_RESET, HIGH);
    //delayMicroseconds(1); // t(TOUT)
    // set control pins
    SPI.begin();
    SPI.setBitOrder(LSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV32);
    digitalWrite(TPI_RESET, LOW);
    pinMode(TPI_RESET, OUTPUT);
    // delay
    delayMicroseconds(SMoGeneral::gDischargeDelay);
    SPI.transfer(0xFF); // activate TPI by emitting 16 or more pulses on TPICLK
    SPI.transfer(0xFF);
}

static void
XPRGDisableTargetTPI(void)
{
    SPI.end();
    // deassert reset
    digitalWrite(TPI_RESET, HIGH);
#if SMO_LAYOUT==SMO_LAYOUT_HVPROG2
    digitalWrite(SMO_HVENABLE, LOW); // disable 12V
#endif
}

/** Busy-waits while the NVM controller is busy performing a NVM operation, such as a FLASH page read.
 *
 *  \return Boolean \c true if the NVM controller became ready within the timeout period, \c false otherwise
 */
void
WaitWhileNVMBusBusy(void)
{
    uint8_t s;
    /* Poll the STATUS register to check to see if NVM access has been enabled */
    do {
        /* Send the SLDCS command to read the TPI STATUS register to see the NVM bus is active */
        XPRGTransfer(TPI_CMD_SLDCS | REG_TPISR, &s);
    } while (!(s & _BV(NVMEN)));
}

/** Waits while the target's NVM controller is busy performing an operation, exiting if the
 *  timeout period expires.
 *
 *  \return Boolean \c true if the NVM controller became ready within the timeout period, \c false otherwise
 */
void
WaitWhileNVMControllerBusy(void)
{
    uint8_t s;
    /* Poll the STATUS register to check to see if NVM access has been enabled */
    do {
        /* Send the SIN command to read the TPI STATUS register to see the NVM bus is busy */
        XPRGTransfer(TPI_CMD_SIN(XPRGParam.NVMCSR), &s);
    } while (s & _BV(NVMBSY));
}
//----------------------------------------------------------------------------------------
static void
EnterProgmode()
{
    /* Enable TPI programming mode with the attached target */
    XPRGEnableTargetTPI();

    /* Direction change guard time to 0 USART bits */
    XPRGTransfer(TPI_CMD_SSTCS | REG_TPIPCR, (uint8_t)(GT2 | GT1 | GT0));

    /* Enable access to the XPROG NVM bus by sending the documented NVM access key to the device */
    XPRGSendKey();

    /* Wait until the NVM bus becomes active */
    WaitWhileNVMBusBusy();
 
 //-----------------------------------------------------------------------------------------------   
    /* determine write size */
    /* there's no way but hardcoding the value for each microcontroller */
    XPRGTransfer(TPI_CMD_SOUT(XPRGParam.NVMCMD), (uint8_t)NO_OPERATION);
    XPRGTransfer(TPI_CMD_SSTPR | 0, 0xC1); // location of the second byte of signature
    XPRGTransfer(TPI_CMD_SSTPR | 1, 0x3F);

    uint8_t c;
    XPRGTransfer(TPI_CMD_SLD | POINTER_UNCHANGED, (uint8_t *)&c);
    switch (c) {
    case 0x91: // possibly t20
        sPageMask = 0x03;
        break;
    case 0x92: // possibly t40
        sPageMask = 0x07;
        break;
    default: // t4, t5, t9, t10
        sPageMask = 0x01;
    }
//-----------------------------------------------------------------------------------------------    
    XPRG_Body[1] = XPRG_ERR_OK;
    sAnswerLen = 2;
}

static void
LeaveProgmode()
{
    WaitWhileNVMBusBusy();

    uint8_t s;
    do {
        /* Clear the NVMEN bit in the TPI STATUS register to disable TPI mode */
        XPRGTransfer(TPI_CMD_SSTCS | REG_TPISR, (uint8_t)0x00);

        /* Read back the STATUS register, check to see if it took effect */
        XPRGTransfer(TPI_CMD_SLDCS | REG_TPISR, &s);
    } while (s);

    XPRGDisableTargetTPI();
    XPRG_Body[1] = XPRG_ERR_OK;
    sAnswerLen = 2;
}

static void
Erase()
{
    uint8_t mode     = XPRG_Body[1];
    //SMoGeneral::gAddress.c[3] = XPRG_Body[2];
    //SMoGeneral::gAddress.c[2] = XPRG_Body[3];
    //SMoGeneral::gAddress.c[1] = XPRG_Body[4];
    //SMoGeneral::gAddress.c[0] = XPRG_Body[5];
  
    XPRGTransfer(TPI_CMD_SOUT(XPRGParam.NVMCMD), (mode == XPRG_ERASE_CHIP ? (uint8_t)CHIP_ERASE : (uint8_t)SECTION_ERASE));

    XPRGTransfer(TPI_CMD_SSTPR | 0, XPRG_Body[5]);
    XPRGTransfer(TPI_CMD_SSTPR | 1, XPRG_Body[4]);
    XPRGTransfer(TPI_CMD_SST | POINTER_UNCHANGED, (uint8_t)0xFF);

    WaitWhileNVMControllerBusy();
    
    XPRG_Body[1] = XPRG_ERR_OK;
    sAnswerLen = 2;
}

static void
WriteMem()
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
    
    XPRGTransfer(TPI_CMD_SOUT(XPRGParam.NVMCMD), (uint8_t)WORD_WRITE);
    if (numBytes & 1)
        data[numBytes++] = 0xFF; // padding

    /* Send the address of the location to write to */
    XPRGTransfer(TPI_CMD_SSTPR | 0, XPRG_Body[6]);
    XPRGTransfer(TPI_CMD_SSTPR | 1, XPRG_Body[5]);

    uint16_t i = 0;
    do {
        do {
            XPRGTransfer(TPI_CMD_SST | POINTER_POST_INC, *data++);
            i++;
            XPRGTransfer(TPI_CMD_SST | POINTER_POST_INC, *data++);
            i++;
            if (i == numBytes) {
                while (i & sPageMask) {
                    XPRGSendIdle();
                    XPRGTransfer(TPI_CMD_SST | POINTER_POST_INC, (uint8_t)0xFF);
                    i++;
                    XPRGTransfer(TPI_CMD_SST | POINTER_POST_INC, (uint8_t)0xFF);
                    i++;
                }
                break;
            }
            if ((i & sPageMask) == 0)
                break;
            XPRGSendIdle();
        } while (1);
        WaitWhileNVMControllerBusy();        
    } while (i < numBytes);

    XPRG_Body[1] = XPRG_ERR_OK;
    sAnswerLen = 2;
}

static void
ReadMem()
{
    //uint8_t memcode = XPRG_Body[1];
    //SMoGeneral::gAddress.c[3] = XPRG_Body[2];
    //SMoGeneral::gAddress.c[2] = XPRG_Body[3];
    //SMoGeneral::gAddress.c[1] = XPRG_Body[4];
    //SMoGeneral::gAddress.c[0] = XPRG_Body[5];
    uint16_t numBytes = (XPRG_Body[6]<<8)|XPRG_Body[7];
    uint8_t *outData = &XPRG_Body[2];
    
    sAnswerLen = numBytes + 2;
    
    /* Set the NVM control register to the NO OP command for memory reading */
    XPRGTransfer(TPI_CMD_SOUT(XPRGParam.NVMCMD), (uint8_t)NO_OPERATION);

    /* Send the address of the location to read from */
    XPRGTransfer(TPI_CMD_SSTPR | 0, XPRG_Body[5]);
    XPRGTransfer(TPI_CMD_SSTPR | 1, XPRG_Body[4]);

    while (numBytes--) {
        XPRGTransfer(TPI_CMD_SLD | POINTER_POST_INC, outData++);
    }
    XPRG_Body[1] = XPRG_ERR_OK;
}

static void
CRC()
{
    XPRG_Body[1] = XPRG_ERR_FAILED;
    sAnswerLen = 5;
}

static void
SetParam()
{
    uint8_t param = XPRG_Body[1];
    
    switch (param) {
    case XPRG_PARAM_NVMCMD_REG:
        XPRGParam.NVMCMD = XPRG_Body[2];
        break;
    case XPRG_PARAM_NVMCSR_REG:
        XPRGParam.NVMCSR = XPRG_Body[2];
        break;       
    default:
        // not implemented
        break;
    }
    XPRG_Body[1] = XPRG_ERR_OK;
    sAnswerLen = 2;
}

void
SMoXPROG::XPROG()
{
    uint8_t XPROGCommand = XPRG_Body[0];

    uint16_t answerlen;
    if (SelectedProtocol != XPRG_MODE_TPI) {
        SMoCommand::SendResponse(STATUS_CMD_FAILED);
        return;
    }
    switch (XPROGCommand) {
    case XPRG_CMD_ENTER_PROGMODE:
        EnterProgmode();
        break;
    case XPRG_CMD_LEAVE_PROGMODE:
        LeaveProgmode();
        break;
    case XPRG_CMD_ERASE:
        Erase();
        break;
    case XPRG_CMD_WRITE_MEM:
        WriteMem();
        break;
    case XPRG_CMD_READ_MEM:
        ReadMem();
        break;
    case XPRG_CMD_CRC:
        CRC();
        break;
    case XPRG_CMD_SET_PARAM:
        SetParam();
        break;
    default:
        SMoCommand::SendResponse(STATUS_CMD_FAILED);
        return;
    }
    SMoCommand::SendResponse(XPROGCommand, sAnswerLen+1);  
}

void
SMoXPROG::XPROG_SetMode()
{
    int8_t mode = XPRG_Body[0];
 
    SelectedProtocol = mode;
    SMoCommand::SendResponse();
}
