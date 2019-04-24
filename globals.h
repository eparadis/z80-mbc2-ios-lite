// ------------------------------------------------------------------------------
//
//  Constants
//
// ------------------------------------------------------------------------------


const String  compTimeStr   = __TIME__;  // Compile timestamp string
const String  compDateStr   = __DATE__;  // Compile datestamp string
const byte    debug         = 0;        // Debug off = 0, on = 1

// boot payloads
#include "boot_payloads.h"

const byte * const bootPh2Table[3] PROGMEM = {boot_A_, boot_B_, boot_C_}; // Payload addresses table
const byte  bootModeAddr  = 10;          // Internal EEPROM address for boot mode storage
const byte  autoexecFlagAddr = 12;       // Internal EEPROM address for AUTOEXEC flag storage
const byte  clockModeAddr = 13;          // Internal EEPROM address for the Z80 clock 4/8 MHz switch
const word  nvStorageAddr = 0x0100;      // Internal EEPROM address for non-volatile storage (256 bytes total)
byte  nvOffset = 0;                // current offset from nvStorageAddr to read from or write to.

// ------------------------------------------------------------------------------
//
//  Global variables
//
// ------------------------------------------------------------------------------

// General purpose variables
byte          ioAddress;                  // I/O virtual address. Only two possible addresses are allowed (0 and 1)
byte          ioData;                     // Data byte used for the I/O operation
byte          ioOpcode       = 0xFF;      // I/O operation code (0xFF means "No Operation")
byte          ioByteCnt;                  // Exchanged bytes counter during an I/O operation
byte          tempByte;
byte          moduleGPIO     = 0;         // Set to 1 if the module is found, 0 otherwise
byte          bootMode       = 0;         // Set the payload array to boot 
                                          // (0: boot_A_[], 1: boot_C_[], 2: boot_B_[])
byte *        BootImage;                  // Pointer to selected payload array (image) to boot
word          BootImageSize;              // Size of the selected payload array (image) to boot
word          BootStrAddr;                // Starting address of the selected payload array (image) to boot
byte          Z80IntEnFlag   = 0;         // Z80 INT_ enable flag (0 = No INT_ used, 1 = INT_ used for I/O)
unsigned long timeStamp;                  // Timestamp for led blinking
char          inChar;                     // Input char from serial
byte          iCount;                     // Temporary variable
byte          clockMode;                  // Z80 clock HI/LO speed selector (0 = 4MHz, 1 = 8MHz)

// CP/M support variables - NOT USED
byte          autoexecFlag;               // Set to 1 if AUTOEXEC must be executed at cold boot, 0 otherwise


void blinkIOSled(unsigned long *timestamp)
// Blink led IOS using a timestamp
{
  if ((millis() - *timestamp) > 200)
  {
    digitalWrite(LED_IOS,!digitalRead(LED_IOS));
    *timestamp = millis();
  }
}


// ------------------------------------------------------------------------------
//
// Hardware definitions for A040618 GPE Option (Optional GPIO Expander)
//
// ------------------------------------------------------------------------------

#define   GPIOEXP_ADDR  0x20  // I2C module address (see datasheet)
#define   IODIRA_REG    0x00  // MCP23017 internal register IODIRA  (see datasheet)
#define   IODIRB_REG    0x01  // MCP23017 internal register IODIRB  (see datasheet)
#define   GPPUA_REG     0x0C  // MCP23017 internal register GPPUA  (see datasheet)
#define   GPPUB_REG     0x0D  // MCP23017 internal register GPPUB  (see datasheet)
#define   GPIOA_REG     0x12  // MCP23017 internal register GPIOA  (see datasheet)
#define   GPIOB_REG     0x13  // MCP23017 internal register GPIOB  (see datasheet)
