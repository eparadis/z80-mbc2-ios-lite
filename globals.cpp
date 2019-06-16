#include <Arduino.h>
#include "boot_payloads.h"
#include "pins.h"
#include "globals.h"

// ------------------------------------------------------------------------------
//
//  Constants
//
// ------------------------------------------------------------------------------

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
