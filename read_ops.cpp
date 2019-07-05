#include <PS2Keyboard.h>

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "read_ops.h"
#include "pins.h"
#include "globals.h"
#include "rtc.h"

extern byte ioData;
extern byte ioAddress;
extern byte ioOpcode;
extern byte tempByte;
extern byte ioByteCnt;
extern byte moduleGPIO;
extern byte autoexecFlag;
extern byte nvOffset;
extern byte foundRTC;
extern byte seconds, minutes, hours, day, month, year, tempC;
extern PS2Keyboard keyboard;

void readOperation() {
  if (!digitalRead(RD_))
    // I/O READ operation requested

    // ----------------------------------------
    // VIRTUAL I/O READ ENGINE
    // ----------------------------------------

  {
    ioAddress = digitalRead(AD0);             // Read Z80 address bus line AD0 (PC2)
    ioData = 0;                               // Clear input data buffer
    if (ioAddress)                            // Check the I/O address (only AD0 is checked!)
      // .........................................................................................................
      //
      // AD0 = 1 (I/O read address = 0x01). SERIAL RX.
      //
      // Execute a Serial I/O Read operation.
      // .........................................................................................................
      //
    {
      //
      // SERIAL RX:
      //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                             D7 D6 D5 D4 D3 D2 D1 D0    ASCII char read from serial
      //
      // NOTE 1: if there is no input char, a value 0xFF is forced as input char.
      // NOTE 2: the INT_ signal is always reset (set to HIGH) after this I/O operation.
      // NOTE 3: This is the only I/O that do not require any previous STORE OPCODE operation (for fast polling)
      //
      ioData = 0xFF;
      if (Serial.available() > 0) ioData = Serial.read();
      if (keyboard.available()) ioData = keyboard.read(); // for now, the keyboard has priority
      digitalWrite(INT_, HIGH);
    }
    else
      // .........................................................................................................
      //
      // AD0 = 0 (I/O read address = 0x00). EXECUTE READ OPCODE.
      //
      // Execute the previously stored I/O read operation with the current data.
      // The code of the I/O operation (Opcode) must be previously stored with a STORE OPCODE operation.
      //
      // NOTE: For multi-byte read opcode (as DATETIME) read sequentially all the data bytes without to send
      //       a STORE OPCODE operation before each data byte after the first one.
      // .........................................................................................................
      //
    {
      switch (ioOpcode)
        // Execute the requested I/O READ Opcode. The 0xFF value is reserved as "No operation".
      {
        case  0x80:
          // USER KEY:
          //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
          //                            ---------------------------------------------------------
          //                              0  0  0  0  0  0  0  0    USER Key not pressed
          //                              0  0  0  0  0  0  0  1    USER Key pressed

          tempByte = digitalRead(USER);         // Save USER led status
          pinMode(USER, INPUT_PULLUP);          // Read USER Key
          ioData = !digitalRead(USER);
          pinMode(USER, OUTPUT);
          digitalWrite(USER, tempByte);         // Restore USER led status
          ioOpcode = 0xFF;                      // All done. Set ioOpcode = "No operation"
          break;

        case  0x81:
          // GPIOA Read (GPE Option):
          //
          //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
          //                            ---------------------------------------------------------
          //                             D7 D6 D5 D4 D3 D2 D1 D0    GPIOA value (see MCP32017 datasheet)
          //
          // NOTE: a value 0x00 is forced if the GPE Option is not present

          if (moduleGPIO)
          {
            // Set MCP23017 pointer to GPIOA
            Wire.beginTransmission(GPIOEXP_ADDR);
            Wire.write(GPIOA_REG);
            Wire.endTransmission();
            // Read GPIOA
            Wire.beginTransmission(GPIOEXP_ADDR);
            Wire.requestFrom(GPIOEXP_ADDR, 1);
            ioData = Wire.read();
          }
          ioOpcode = 0xFF;                      // All done. Set ioOpcode = "No operation"
          break;

        case  0x82:
          // GPIOB Read (GPE Option):
          //
          //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
          //                            ---------------------------------------------------------
          //                             D7 D6 D5 D4 D3 D2 D1 D0    GPIOB value (see MCP32017 datasheet)
          //
          // NOTE: a value 0x00 is forced if the GPE Option is not present

          if (moduleGPIO)
          {
            // Set MCP23017 pointer to GPIOB
            Wire.beginTransmission(GPIOEXP_ADDR);
            Wire.write(GPIOB_REG);
            Wire.endTransmission();
            // Read GPIOB
            Wire.beginTransmission(GPIOEXP_ADDR);
            Wire.requestFrom(GPIOEXP_ADDR, 1);
            ioData = Wire.read();
          }
          ioOpcode = 0xFF;                      // All done. Set ioOpcode = "No operation"
          break;

        case  0x83:
          // SYSFLAGS (Various system flags for the OS):
          //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
          //                            ---------------------------------------------------------
          //                              X  X  X  X  X  X  X  0    AUTOEXEC not enabled
          //                              X  X  X  X  X  X  X  1    AUTOEXEC enabled
          //                              X  X  X  X  X  X  0  X    DS3231 RTC not found
          //                              X  X  X  X  X  X  1  X    DS3231 RTC found
          //
          // NOTE: Currently only D1 is used

          ioData = autoexecFlag | (foundRTC << 1);
          ioOpcode = 0xFF;                      // All done. Set ioOpcode = "No operation"
          break;

        case  0x84:
          // DATETIME (Read date/time and temperature from the RTC. Binary values):
          //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
          //                            ---------------------------------------------------------
          //                I/O DATA 0   D7 D6 D5 D4 D3 D2 D1 D0    seconds [0..59]     (1st data byte)
          //                I/O DATA 1   D7 D6 D5 D4 D3 D2 D1 D0    minutes [0..59]
          //                I/O DATA 2   D7 D6 D5 D4 D3 D2 D1 D0    hours   [0..23]
          //                I/O DATA 3   D7 D6 D5 D4 D3 D2 D1 D0    day     [1..31]
          //                I/O DATA 4   D7 D6 D5 D4 D3 D2 D1 D0    month   [1..12]
          //                I/O DATA 5   D7 D6 D5 D4 D3 D2 D1 D0    year    [0..99]
          //                I/O DATA 6   D7 D6 D5 D4 D3 D2 D1 D0    tempC   [-128..127] (7th data byte)
          //
          // NOTE 1: If RTC is not found all read values wil be = 0
          // NOTE 2: Overread data (more then 6 bytes read) will be = 0
          // NOTE 3: The temperature (Celsius) is a byte with two complement binary format [-128..127]

          if (foundRTC)
          {
            if (ioByteCnt == 0) readRTC(&seconds, &minutes, &hours, &day, &month, &year, &tempC); // Read from RTC
            if (ioByteCnt < 7)
              // Send date/time (binary values) to Z80 bus
            {
              switch (ioByteCnt)
              {
                case 0: ioData = seconds; break;
                case 1: ioData = minutes; break;
                case 2: ioData = hours; break;
                case 3: ioData = day; break;
                case 4: ioData = month; break;
                case 5: ioData = year; break;
                case 6: ioData = tempC; break;
              }
              ioByteCnt++;
            }
            else ioOpcode = 0xFF;              // All done. Set ioOpcode = "No operation"
          }
          else ioOpcode = 0xFF;                 // Nothing to do. Set ioOpcode = "No operation"
          break;
        case 0x85:
          // read EEPROM at cursor (Atmega internal EEPROM)
          ioData = EEPROM.read(nvStorageAddr + (nvOffset & 0xFF));
          ioOpcode = 0xFF;              // All done. Set ioOpcode = "No operation"
          break;
      }
    }
    DDRA = 0xFF;                              // Configure Z80 data bus D0-D7 (PA0-PA7) as output
    PORTA = ioData;                           // Current output on data bus

    // Control bus sequence to exit from a wait state (M I/O read cycle)
    digitalWrite(BUSREQ_, LOW);               // Request for a DMA
    digitalWrite(WAIT_RES_, LOW);             // Now is safe reset WAIT FF (exiting from WAIT state)
    delayMicroseconds(2);                     // Wait 2us just to be sure that Z80 read the data and go HiZ
    DDRA = 0x00;                              // Configure Z80 data bus D0-D7 (PA0-PA7) as input with pull-up
    PORTA = 0xFF;
    digitalWrite(WAIT_RES_, HIGH);            // Now Z80 is in DMA (HiZ), so it's safe set WAIT_RES_ HIGH again
    digitalWrite(BUSREQ_, HIGH);              // Resume Z80 from DMA
  }
  else
    // INTERRUPT operation setting IORQ_ LOW, so nothing to do

    // ----------------------------------------
    // VIRTUAL INTERRUPT
    // ----------------------------------------

    // Nothing to do
  {
    //
    // DEBUG ----------------------------------
    if (debug)
    {
      Serial.println();
      Serial.println("DEBUG: INT operation (nothing to do)");
    }
    // DEBUG END ------------------------------
    //

    // Control bus sequence to exit from a wait state (M interrupt cycle)
    digitalWrite(BUSREQ_, LOW);               // Request for a DMA
    digitalWrite(WAIT_RES_, LOW);             // Reset WAIT FF exiting from WAIT state
    digitalWrite(WAIT_RES_, HIGH);            // Now Z80 is in DMA, so it's safe set WAIT_RES_ HIGH again
    digitalWrite(BUSREQ_, HIGH);              // Resume Z80 from DMA
  }
}
