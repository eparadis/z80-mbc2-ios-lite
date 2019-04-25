#include <Arduino.h> // added for VS Code support

#include "pins.h"
#include "globals.h"
#include "boot.h"
#include "rtc.h"
#include "write_ops.h"
#include "read_ops.h"

#include <avr/pgmspace.h>                 // Needed for PROGMEM
#include "Wire.h"                         // Needed for I2C bus
#include <EEPROM.h>                       // Needed for internal EEPROM R/W

/*
  S220618 - HW ref: A040618

  IOS-LITE - I/O Subsystem for Z80-MBC2 (Multi Boot Computer) - LITE Edition (No SD disk support)
  (Z80 128kB RAM @ 4/8Mhz)

  Notes:

  1:  This SW is ONLY for the Atmega used as EEPROM and I/O subsystem (16MHz external oscillator).
    It is currently targeted to the Atmega644P or similar.
    https://github.com/MCUdude/MightyCore is required

  2:  Tested on Atmega32A @ Arduino IDE 1.8.5.

  3:  Embedded FW: S210718 uBIOS + Basic, S200718 iLoad (Intel-Hex loader),
                 Forth interpreter

  4:  Utilities:   S111216 TASM conversion utility

  CHANGELOG:
  S220618           First release.
*/

void setup()
{
  byte          data;                       // External RAM data byte
  word          address;                    // External RAM current address;
  byte          bootSelection = 0;          // Flag to enter into the boot mode selection

  // Check USER Key for boot mode changes
  pinMode(USER, INPUT_PULLUP);                    // Read USER Key to enter into the boot mode selection
  if (!digitalRead(USER)) bootSelection = 1;

  initializePins();
  clockMode = lookupClockMode();
  searchForI2CDevices();
  printBootMessageHeader();
  reportDiscoveredI2CDevices();
  bootMenu(bootSelection);
  bootZ80();
}

void loop()
{
  if (!digitalRead(WAIT_))     // I/O operaton requested
  {
    if (!digitalRead(WR_))
      writeOperation();
    else
      readOperation();
  }
}

void initializePins() {
  // Initialize RESET_ and WAIT_RES_
  pinMode(RESET_, OUTPUT);                        // Configure RESET_ and set it ACTIVE
  digitalWrite(RESET_, LOW);
  pinMode(WAIT_RES_, OUTPUT);                     // Configure WAIT_RES_ and set it ACTIVE to reset the WAIT FF (U1C)
  digitalWrite(WAIT_RES_, LOW);

  // Initialize USER,  INT_, RAM_CE2, and BUSREQ_
  pinMode(USER, OUTPUT);                          // USER led OFF
  digitalWrite(USER, HIGH);
  pinMode(INT_, INPUT_PULLUP);                    // Configure INT_ and set it NOT ACTIVE
  pinMode(INT_, OUTPUT);
  digitalWrite(INT_, HIGH);
  pinMode(RAM_CE2, OUTPUT);                       // Configure RAM_CE2 as output
  digitalWrite(RAM_CE2, HIGH);                    // Set RAM_CE2 active
  pinMode(WAIT_, INPUT);                          // Configure WAIT_ as input
  pinMode(BUSREQ_, INPUT_PULLUP);                 // Set BUSREQ_ HIGH
  pinMode(BUSREQ_, OUTPUT);
  digitalWrite(BUSREQ_, HIGH);

  // Initialize D0-D7, AD0, MREQ_, RD_ and WR_
  DDRA = 0x00;                                    // Configure Z80 data bus D0-D7 (PA0-PA7) as input with pull-up
  PORTA = 0xFF;
  pinMode(MREQ_, INPUT_PULLUP);                   // Configure MREQ_ as input with pull-up
  pinMode(RD_, INPUT_PULLUP);                     // Configure RD_ as input with pull-up
  pinMode(WR_, INPUT_PULLUP);                     // Configure WR_ as input with pull-up
  pinMode(AD0, INPUT_PULLUP);

  // Initialize the Logical RAM Bank (32KB) to map into the lower half of the Z80 addressing space
  pinMode(BANK0, OUTPUT);                         // Set RAM Logical Bank 1
  digitalWrite(BANK0, HIGH);
  pinMode(BANK1, OUTPUT);
  digitalWrite(BANK1, LOW);

  // Initialize (park) SPI pins
  pinMode(SS_, INPUT);                            // SD SPI parked (not used)
  pinMode(MISO, INPUT);
  pinMode(MOSI, INPUT_PULLUP);
  pinMode(SCK, INPUT);

  // Initialize CLK (single clock pulses mode) and reset the Z80 CPU
  pinMode(CLK, OUTPUT);                           // Set CLK as output
  digitalWrite(CLK, LOW);
  singlePulsesResetZ80();                         // Reset the Z80 CPU using single clock pulses

  // Initialize (park) MCU_RTS and MCU_CTS
  pinMode(MCU_RTS_, INPUT_PULLUP);                // Parked (not used)
  pinMode(MCU_CTS_, INPUT_PULLUP);
}

byte lookupClockMode() {
  // Read the Z80 CPU speed mode
  byte orig = EEPROM.read(clockModeAddr);
  if (orig > 1)             // Check if it is a valid value, otherwise set it to 4MHz mode
  {
    EEPROM.write(clockModeAddr, 1);
    return 1;
  }
  return orig;         // Read the previous stored value
}

void printBootMessageHeader() {
  // Print some system information
  Serial.begin(9600);
  Serial.println();
  Serial.println("Z80-MBC2 - A040618");
  Serial.println("IOS-LITE - I/O Subsystem - S220618");
  Serial.println();

  // Print the Z80 clock speed mode
  Serial.print("IOS: Z80 clock set at ");
  Serial.print(8 - (clockMode * 4));
  Serial.println("MHz");
}

void searchForI2CDevices() {
  // Initialize the EXP_PORT (I2C) and search for "known" optional modules
  Wire.begin();                                   // Wake up I2C bus
  Wire.beginTransmission(GPIOEXP_ADDR);
  if (Wire.endTransmission() == 0) moduleGPIO = 1;// Set to 1 if GPIO Module is found
}

void reportDiscoveredI2CDevices() {
  // Print RTC and GPIO informations if found
  foundRTC = autoSetRTC();                        // Check if RTC is present and initialize it as needed
  if (moduleGPIO) Serial.println("IOS: Found GPE Option");
}

void bootMenu(byte bootSelection) {

  char          minBootChar   = '1';        // Minimum allowed ASCII value selection (boot selection)
  char          maxSelChar    = '5';        // Maximum allowed ASCII value selection (boot selection)
  byte          maxBootMode   = 3;          // Default maximum allowed value for bootMode [0..3]
  // ----------------------------------------
  // BOOT SELECTION AND SYS PARAMETERS MENU
  // ----------------------------------------

  // Boot selection and system parameters menu if requested

  bootMode = EEPROM.read(bootModeAddr);           // Read the previous stored boot mode
  if ((bootSelection == 1 ) || (bootMode > maxBootMode))
    // Enter in the boot selection menu if USER key was pressed at startup
    //   or an invalid bootMode code was read from internal EEPROM
  {
    while (Serial.available() > 0)                // Flush input serial Rx buffer
    {
      Serial.read();
    }
    Serial.println();
    Serial.println("IOS: Select boot mode or system parameters:");
    Serial.println();
    if (bootMode <= maxBootMode)
      // Previous valid boot mode read, so enable '0' selection
    {
      minBootChar = '0';
      Serial.print(" 0: No change (->");
      Serial.print(bootMode + 1);
      Serial.println(")");
    }
    Serial.println(" 1: Basic");
    Serial.println(" 2: Forth");
    Serial.println(" 3: iLoad");
    Serial.println(" 4: EEPROM scratchpad");
    Serial.println(" 5: Change Z80 clock speed (4/8MHz)");
    if (foundRTC)
      // RTC module is present, so add a choice
    {
      Serial.println(" 6: Change RTC time/date");
      maxSelChar = '6';
    }

    // Ask a choice
    Serial.println();
    timeStamp = millis();
    Serial.print("Enter your choice >");
    do
    {
      blinkIOSled(&timeStamp);
      inChar = Serial.read();
    }
    while ((inChar < minBootChar) || (inChar > maxSelChar));
    Serial.print(inChar);
    Serial.println("  Ok");

    // Make the selected action
    if (inChar == '5')
      // Change the clock speed of the Z80 CPU
    {
      clockMode = !clockMode;                     // Toggle speed mode (4/8MHz)
      EEPROM.write(clockModeAddr, clockMode);     // Save it to the internal EEPROM
    }
    if (inChar == '6') ChangeRTC();               // Change RTC Date/Time
    bootMode = inChar - '1';                      // Calculate bootMode from inChar
    if (bootMode < 4) EEPROM.write(bootModeAddr, bootMode); // Save to the internal EEPROM if required
    else bootMode = EEPROM.read(bootModeAddr);    // Reload boot mode if '0' or > '4' choice selected
  }
}
