#include <PS2Keyboard.h>

#include "pins.h"
#include "globals.h"
#include "boot.h"
#include "rtc.h"
#include "write_ops.h"
#include "read_ops.h"
#include "boot_payloads.h"

#include <avr/pgmspace.h>                 // Needed for PROGMEM
#include "Wire.h"                         // Needed for I2C bus
#include <EEPROM.h>                       // Needed for internal EEPROM R/W

extern unsigned long timeStamp;
extern byte moduleGPIO;
extern char inChar;
extern byte foundRTC;

extern byte clockMode;
extern byte bootMode;

extern byte *BootImage;
extern word BootImageSize;
extern const byte * const bootPh2Table[3] PROGMEM;
extern word BootStrAddr;
extern byte Z80IntEnFlag;

extern PS2Keyboard keyboard;

const byte    JP_nn        =  0xC3;     // Opcode of the Z80 instruction: JP nn


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

  // ------------------------------------------------------------------------------
  //
  //  Local variables
  //
  // ------------------------------------------------------------------------------

  byte          data;                       // External RAM data byte
  word          address;                    // External RAM current address;
  char          minBootChar   = '1';        // Minimum allowed ASCII value selection (boot selection)
  char          maxSelChar    = '4';        // Maximum allowed ASCII value selection (boot selection)
  byte          maxBootMode   = 2;          // Default maximum allowed value for bootMode [0..2]
  byte          bootSelection = 0;          // Flag to enter into the boot mode selection

  // ------------------------------------------------------------------------------

  // ----------------------------------------
  // INITIALITATION
  // ----------------------------------------

  // Initialize RESET_ and WAIT_RES_
  pinMode(RESET_, OUTPUT);                        // Configure RESET_ and set it ACTIVE
  digitalWrite(RESET_, LOW);
  pinMode(WAIT_RES_, OUTPUT);                     // Configure WAIT_RES_ and set it ACTIVE to reset the WAIT FF (U1C)
  digitalWrite(WAIT_RES_, LOW);

  // Check USER Key for boot mode changes
  pinMode(USER, INPUT_PULLUP);                    // Read USER Key to enter into the boot mode selection
  if (!digitalRead(USER)) bootSelection = 1;

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

  // Initialize PS2 Keyboard attached to "MCU_RTS and MCU_CTS"
  keyboard.begin(MCU_RTS_, MCU_CTS_);
  //pinMode(MCU_RTS_, INPUT_PULLUP);
  //pinMode(MCU_CTS_, INPUT_PULLUP);

  // Read the Z80 CPU speed mode
  if (EEPROM.read(clockModeAddr) > 1)             // Check if it is a valid value, otherwise set it to 4MHz mode
    // Not a valid value. Set it to 4MHz.
  {
    EEPROM.write(clockModeAddr, 1);
  }
  clockMode = EEPROM.read(clockModeAddr);         // Read the previous stored value

  // Initialize the EXP_PORT (I2C) and search for "known" optional modules
  Wire.begin();                                   // Wake up I2C bus
  Wire.beginTransmission(GPIOEXP_ADDR);
  if (Wire.endTransmission() == 0) moduleGPIO = 1;// Set to 1 if GPIO Module is found

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

  // Print RTC and GPIO informations if found
  foundRTC = autoSetRTC();                        // Check if RTC is present and initialize it as needed
  if (moduleGPIO) Serial.println("IOS: Found GPE Option");

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
    Serial.println(" 4: Change Z80 clock speed (4/8MHz)");
    if (foundRTC)
      // RTC module is present, so add a choice
    {
      Serial.println(" 5: Change RTC time/date");
      maxSelChar = '5';
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
    if (inChar == '4')
      // Change the clock speed of the Z80 CPU
    {
      clockMode = !clockMode;                     // Toggle speed mode (4/8MHz)
      EEPROM.write(clockModeAddr, clockMode);     // Save it to the internal EEPROM
    }
    if (inChar == '5') ChangeRTC();               // Change RTC Date/Time
    bootMode = inChar - '1';                      // Calculate bootMode from inChar
    if (bootMode < 3) EEPROM.write(bootModeAddr, bootMode); // Save to the internal EEPROM if required
    else bootMode = EEPROM.read(bootModeAddr);    // Reload boot mode if '0' or > '3' choice selected
  }

  // ----------------------------------------
  // Z80 PROGRAM BOOT
  // ----------------------------------------

  // Get the address of the payload array in PROGMEM for the program boot and its size
  switch (bootMode)
  {
    case 0:                                       // Basic boot
      BootImage = (byte *) pgm_read_word (&bootPh2Table[0]);
      BootImageSize = sizeof(boot_A_);
      BootStrAddr = boot_A_StrAddr;
      Z80IntEnFlag = 1;                           // Enable INT_ signal generation (Z80 M1 INT I/O)
      break;
    case 1:                                       // Forth boot
      BootImage = (byte *) pgm_read_word (&bootPh2Table[2]);
      BootImageSize = sizeof(boot_C_);
      BootStrAddr = boot_C_StrAddr;
      break;
    case 2:                                       // iLoad boot
      BootImage = (byte *) pgm_read_word (&bootPh2Table[1]);
      BootImageSize = sizeof(boot_B_);
      BootStrAddr = boot_B_StrAddr;
      break;
  }
  digitalWrite(WAIT_RES_, HIGH);                // Set WAIT_RES_ HIGH (Led LED_0 ON)

  // Load a JP instruction if the boot program starting addr is > 0x0000
  if (BootStrAddr > 0x0000)                     // Check if the boot program starting addr > 0x0000
    // Inject a "JP <BootStrAddr>" instruction to jump at boot starting address
  {
    loadHL(0x0000);                             // HL = 0x0000 (used as pointer to RAM)
    loadByteToRAM(JP_nn);                       // Write the JP opcode @ 0x0000;
    loadByteToRAM(lowByte(BootStrAddr));        // Write LSB to jump @ 0x0001
    loadByteToRAM(highByte(BootStrAddr));       // Write MSB to jump @ 0x0002
    //
    // DEBUG ----------------------------------
    if (debug)
    {
      Serial.print("DEBUG: Injected JP 0x");
      Serial.println(BootStrAddr, HEX);
    }
    // DEBUG END ------------------------------
    //
  }

  // Execute the boot
  loadHL(BootStrAddr);                           // Set Z80 HL = boot starting address (used as pointer to RAM);
  //
  // DEBUG ----------------------------------
  if (debug)
  {
    Serial.print("DEBUG: BootImageSize = ");
    Serial.println(BootImageSize);
    Serial.print("DEBUG: BootStrAddr = ");
    Serial.println(BootStrAddr, HEX);
  }
  // DEBUG END ------------------------------
  //
  Serial.print("IOS: Loading boot program...");
  for (word i = 0; i < BootImageSize; i++)
    // Write boot program into external RAM
  {
    loadByteToRAM(pgm_read_byte(BootImage + i));  // Write current data byte into RAM
  }
  Serial.println(" Done");
  digitalWrite(RESET_, LOW);                      // Activate the RESET_ signal

  // Initialize CLK @ 4/8MHz. Z80 clock_freq = (Atmega_clock) / ((OCR2 + 1) * 2)
  ASSR &= ~(1 << AS2);                            // Set Timer2 clock from system clock

  TCCR2B |= (1 << CS20);                     // set CS20          // Set Timer2 clock to "no prescaling"
  TCCR2B &= ~((1 << CS21) | (1 << CS22));    // clr CS21 and CS22
  TCCR2A |= (1 << WGM21);                    // set WGM21         // Set Timer2 CTC mode
  TCCR2A &= ~(1 << WGM20);                   // clr WGM20
  TCCR2B &= ~(1 << WGM22);                   // clr WGM22
  TCCR2A |= (1 <<  COM2A0);                  // set COM2A0        // Set "toggle OC2 on compare match"
  TCCR2A &= ~(1 << COM2A1);                  // clr COM2A1
  OCR2A = clockMode;                         // Set the compare value to toggle OC2 (0 = 8MHz or 1 = 4MHz)

  pinMode(CLK, OUTPUT);                           // Set OC2 as output and start to output the clock @ 4Mhz
  Serial.println("IOS: Z80 is running from now");
  Serial.println();

  // Flush input serial Rx buffer
  while (Serial.available() > 0)
  {
    Serial.read();
  }

  // Leave the Z80 CPU running
  delay(1);                                       // Just to be sure...
  digitalWrite(RESET_, HIGH);                     // Release Z80 from reset and let it run
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

  // for now, poll they keyboard buffer to see if we need to interrupt
  if(keyboard.available() && Z80IntEnFlag) {
    digitalWrite(INT_, LOW);
  }
}

void serialEvent()
// Set INT_ to ACTIVE if there are received chars from serial to read and if the interrupt generation is enabled
{
  if ((Serial.available()) && Z80IntEnFlag) digitalWrite(INT_, LOW);
}
