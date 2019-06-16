#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include "pins.h"
#include "globals.h"

extern byte ioOpcode;
extern byte ioAddress;
extern byte ioData;
extern byte ioByteCnt;
extern byte moduleGPIO;
extern byte nvOffset;

void storeWriteOpcode() {
  // .........................................................................................................
  //
  // AD0 = 1 (I/O write address = 0x01). STORE OPCODE.
  //
  // Store (write) an "I/O operation code" (Opcode) and reset the exchanged bytes counter.
  //
  // NOTE 1: An Opcode can be a write or read Opcode, if the I/O operation is read or write.
  // NOTE 2: the STORE OPCODE operation must always precede an EXECUTE WRITE OPCODE or EXECUTE READ OPCODE
  //         operation.
  // NOTE 3: For multi-byte read opcode (as DATETIME) read sequentially all the data bytes without to send
  //         a STORE OPCODE operation before each data byte after the first one.
  // .........................................................................................................
  //
  // Currently defined Opcodes for I/O write operations:
  //
  //   Opcode     Name            Exchanged bytes
  // -------------------------------------------------
  // Opcode 0x00  USER LED        1
  // Opcode 0x01  SERIAL TX       1
  // Opcode 0x03  GPIOA Write     1
  // Opcode 0x04  GPIOB Write     1
  // Opcode 0x05  IODIRA Write    1
  // Opcode 0x06  IODIRB Write    1
  // Opcode 0x07  GPPUA Write     1
  // Opcode 0x08  GPPUB Write     1
  // Opcode 0x09  set EEPROM cursor         1
  // Opcode 0x0A  write EEPROM at cursor    1
  // Opcode 0xFF  No operation    1
  //
  //
  // Currently defined Opcodes for I/O read operations:
  //
  //   Opcode     Name            Exchanged bytes
  // -------------------------------------------------
  // Opcode 0x80  USER KEY        1
  // Opcode 0x81  GPIOA Read      1
  // Opcode 0x82  GPIOB Read      1
  // Opcode 0x83  SYSFLAGS        1
  // Opcode 0x84  DATETIME        7
  // Opcode 0x85  read EEPROM at cursor  1
  // Opcode 0xFF  No operation    1
  //
  // See the following lines for the Opcodes details.
  //
  // .........................................................................................................
  {
    ioOpcode = ioData;                        // Store the I/O operation code (Opcode)
    ioByteCnt = 0;                            // Reset the exchanged bytes counter
  }
}

void executeWriteOpcode() {
  // .........................................................................................................
  //
  // AD0 = 0 (I/O write address = 0x00). EXECUTE WRITE OPCODE.
  //
  // Execute the previously stored I/O write opcode with the current data.
  // The code of the I/O write operation (Opcode) must be previously stored with a STORE OPCODE operation.
  // .........................................................................................................
  //
  {
    switch (ioOpcode)
      // Execute the requested I/O WRITE Opcode. The 0xFF value is reserved as "No operation".
    {
      case  0x00:
        // USER LED:
        //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
        //                            ---------------------------------------------------------
        //                              x  x  x  x  x  x  x  0    USER Led off
        //                              x  x  x  x  x  x  x  1    USER Led on

        if (ioData & B00000001) digitalWrite(USER, LOW);
        else digitalWrite(USER, HIGH);
        break;

      case  0x01:
        // SERIAL TX:
        //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
        //                            ---------------------------------------------------------
        //                             D7 D6 D5 D4 D3 D2 D1 D0    ASCII char to be sent to serial

        Serial.write(ioData);
        break;

      case  0x03:
        // GPIOA Write (GPE Option):
        //
        //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
        //                            ---------------------------------------------------------
        //                             D7 D6 D5 D4 D3 D2 D1 D0    GPIOA value (see MCP32017 datasheet)

        if (moduleGPIO)
        {
          Wire.beginTransmission(GPIOEXP_ADDR);
          Wire.write(GPIOA_REG);                // Select GPIOA
          Wire.write(ioData);                   // Write value
          Wire.endTransmission();
        }
        break;

      case  0x04:
        // GPIOB Write (GPE Option):
        //
        //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
        //                            ---------------------------------------------------------
        //                             D7 D6 D5 D4 D3 D2 D1 D0    GPIOB value (see MCP32017 datasheet)

        if (moduleGPIO)
        {
          Wire.beginTransmission(GPIOEXP_ADDR);
          Wire.write(GPIOB_REG);                // Select GPIOB
          Wire.write(ioData);                   // Write value
          Wire.endTransmission();
        }
        break;

      case  0x05:
        // IODIRA Write (GPE Option):
        //
        //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
        //                            ---------------------------------------------------------
        //                             D7 D6 D5 D4 D3 D2 D1 D0    IODIRA value (see MCP32017 datasheet)

        if (moduleGPIO)
        {
          Wire.beginTransmission(GPIOEXP_ADDR);
          Wire.write(IODIRA_REG);               // Select IODIRA
          Wire.write(ioData);                   // Write value
          Wire.endTransmission();
        }
        break;

      case  0x06:
        // IODIRB Write (GPE Option):
        //
        //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
        //                            ---------------------------------------------------------
        //                             D7 D6 D5 D4 D3 D2 D1 D0    IODIRB value (see MCP32017 datasheet)

        if (moduleGPIO)
        {
          Wire.beginTransmission(GPIOEXP_ADDR);
          Wire.write(IODIRB_REG);               // Select IODIRB
          Wire.write(ioData);                   // Write value
          Wire.endTransmission();
        }
        break;

      case  0x07:
        // GPPUA Write (GPE Option):
        //
        //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
        //                            ---------------------------------------------------------
        //                             D7 D6 D5 D4 D3 D2 D1 D0    GPPUA value (see MCP32017 datasheet)

        if (moduleGPIO)
        {
          Wire.beginTransmission(GPIOEXP_ADDR);
          Wire.write(GPPUA_REG);                // Select GPPUA
          Wire.write(ioData);                   // Write value
          Wire.endTransmission();
        }
        break;

      case  0x08:
        // GPPUB Write (GPIO Exp. Mod. ):
        //
        //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
        //                            ---------------------------------------------------------
        //                             D7 D6 D5 D4 D3 D2 D1 D0    GPPUB value (see MCP32017 datasheet)

        if (moduleGPIO)
        {
          Wire.beginTransmission(GPIOEXP_ADDR);
          Wire.write(GPPUB_REG);                // Select GPPUB
          Wire.write(ioData);                   // Write value
          Wire.endTransmission();
        }
        break;

      case 0x09:
        // set EEPROM R/W cursor (Atmega internal EEPROM)
        nvOffset = ioData;
        break;
      case 0x0A:
        // write a byte to EEPROM at current EEPROM cursor location
        EEPROM.update(nvStorageAddr + (nvOffset & 0xFF), ioData);
    }
    ioOpcode = 0xFF;                // All done. Set ioOpcode = "No operation"
  }
}

void writeOperation() {
  ioAddress = digitalRead(AD0);               // Read Z80 address bus line AD0 (PC2)
  ioData = PINA;                              // Read Z80 data bus D0-D7 (PA0-PA7)
  if (ioAddress)                              // Check the I/O address (only AD0 is checked!)
    storeWriteOpcode();
  else
    executeWriteOpcode();

  // Control bus sequence to exit from a wait state (M I/O write cycle)
  digitalWrite(BUSREQ_, LOW);                 // Request for a DMA
  digitalWrite(WAIT_RES_, LOW);               // Reset WAIT FF exiting from WAIT state
  digitalWrite(WAIT_RES_, HIGH);              // Now Z80 is in DMA, so it's safe set WAIT_RES_ HIGH again
  digitalWrite(BUSREQ_, HIGH);                // Resume Z80 from DMA
}
