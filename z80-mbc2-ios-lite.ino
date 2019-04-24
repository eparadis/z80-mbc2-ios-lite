#include "pins.h"
#include "globals.h"
#include "boot.h"
#include "rtc.h"

#include <avr/pgmspace.h>                 // Needed for PROGMEM
#include "Wire.h"                         // Needed for I2C bus
#include <EEPROM.h>                       // Needed for internal EEPROM R/W

/* ------------------------------------------------------------------------------

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


  --------------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------------

  CHANGELOG:


  S220618           First release.


  --------------------------------------------------------------------------------- */



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

// ------------------------------------------------------------------------------

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

  // Initialize (park) MCU_RTS and MCU_CTS
  pinMode(MCU_RTS_, INPUT_PULLUP);                // Parked (not used)
  pinMode(MCU_CTS_, INPUT_PULLUP);

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

// ------------------------------------------------------------------------------

void loop()
{
  if (!digitalRead(WAIT_))
    // I/O operaton requested
  {
    if (!digitalRead(WR_))
      writeOperation();
    else
      readOperation();
  }
}




// ------------------------------------------------------------------------------

void writeOperation() {
  // I/O WRITE operation requested

  // ----------------------------------------
  // VIRTUAL I/O WRTE ENGINE
  // ----------------------------------------


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

// ------------------------------------------------------------------------------

// Generic routines

// ------------------------------------------------------------------------------

//void printBinaryByte(byte value)
//{
//  for (byte mask = 0x80; mask; mask >>= 1)
//  {
//    Serial.print((mask & value) ? '1' : '0');
//  }
//}

// ------------------------------------------------------------------------------

void serialEvent()
// Set INT_ to ACTIVE if there are received chars from serial to read and if the interrupt generation is enabled
{
  if ((Serial.available()) && Z80IntEnFlag) digitalWrite(INT_, LOW);
}
