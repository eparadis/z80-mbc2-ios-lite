#include "pins.h"
#include <EEPROM.h>

const byte    LD_HL        =  0x36;     // Opcode of the Z80 instruction: LD(HL), n
const byte    INC_HL       =  0x23;     // Opcode of the Z80 instruction: INC HL
const byte    LD_HLnn      =  0x21;     // Opcode of the Z80 instruction: LD HL, nn
const byte    JP_nn        =  0xC3;     // Opcode of the Z80 instruction: JP nn

// ------------------------------------------------------------------------------

// Z80 boot routines

// ------------------------------------------------------------------------------


void pulseClock(byte numPulse)
// Generate <numPulse> clock pulses on the Z80 clock pin.
// The steady clock level is LOW, e.g. one clock pulse is a 0-1-0 transition
{
  for (iCount = 0; iCount < numPulse; iCount++)
  // Generate one clock pulse
  {
    // Send one impulse (0-1-0) on the CLK output
    digitalWrite(CLK, HIGH);
    digitalWrite(CLK, LOW);
  }
}

// ------------------------------------------------------------------------------

void loadByteToRAM(byte value)
// Load a given byte to RAM using a sequence of two Z80 instructions forced on the data bus.
// The RAM_CE2 signal is used to force the RAM in HiZ, so the Atmega can write the needed instruction/data
// on the data bus. Controlling the clock signal and knowing exactly how many clocks pulse are required it is possible control the
// whole loading process.
// In the following "T" are the T-cycles of the Z80 (See the Z80 datashet).
// The two instruction are "LD (HL), n" and "INC (HL)".
{
  // Execute the LD(HL),n instruction (T = 4+3+3). See the Z80 datasheet and manual.
  // After the execution of this instruction the <value> byte is loaded in the memory address pointed by HL.
  pulseClock(1);                      // Execute the T1 cycle of M1 (Opcode Fetch machine cycle)
  digitalWrite(RAM_CE2, LOW);         // Force the RAM in HiZ (CE2 = LOW)
  DDRA = 0xFF;                        // Configure Z80 data bus D0-D7 (PA0-PA7) as output
  PORTA = LD_HL;                      // Write "LD (HL), n" opcode on data bus
  pulseClock(2);                      // Execute T2 and T3 cycles of M1
  DDRA = 0x00;                        // Configure Z80 data bus D0-D7 (PA0-PA7) as input... 
  PORTA = 0xFF;                       // ...with pull-up
  pulseClock(2);                      // Complete the execution of M1 and execute the T1 cycle of the following 
                                      // Memory Read machine cycle
  DDRA = 0xFF;                        // Configure Z80 data bus D0-D7 (PA0-PA7) as output
  PORTA = value;                      // Write the byte to load in RAM on data bus
  pulseClock(2);                      // Execute the T2 and T3 cycles of the Memory Read machine cycle
  DDRA = 0x00;                        // Configure Z80 data bus D0-D7 (PA0-PA7) as input... 
  PORTA = 0xFF;                       // ...with pull-up
  digitalWrite(RAM_CE2, HIGH);        // Enable the RAM again (CE2 = HIGH)
  pulseClock(3);                      // Execute all the following Memory Write machine cycle

  // Execute the INC(HL) instruction (T = 6). See the Z80 datasheet and manual.
  // After the execution of this instruction HL points to the next memory address.
  pulseClock(1);                      // Execute the T1 cycle of M1 (Opcode Fetch machine cycle)
  digitalWrite(RAM_CE2, LOW);         // Force the RAM in HiZ (CE2 = LOW)
  DDRA = 0xFF;                        // Configure Z80 data bus D0-D7 (PA0-PA7) as output
  PORTA = INC_HL;                     // Write "INC(HL)" opcode on data bus
  pulseClock(2);                      // Execute T2 and T3 cycles of M1
  DDRA = 0x00;                        // Configure Z80 data bus D0-D7 (PA0-PA7) as input... 
  PORTA = 0xFF;                       // ...with pull-up
  digitalWrite(RAM_CE2, HIGH);        // Enable the RAM again (CE2 = HIGH)
  pulseClock(3);                      // Execute all the remaining T cycles
}

// ------------------------------------------------------------------------------

void loadHL(word value)
// Load "value" word into the HL registers inside the Z80 CPU, using the "LD HL,nn" instruction.
// In the following "T" are the T-cycles of the Z80 (See the Z80 datashet).
{
  // Execute the LD dd,nn instruction (T = 4+3+3), with dd = HL and nn = value. See the Z80 datasheet and manual.
  // After the execution of this instruction the word "value" (16bit) is loaded into HL.
  pulseClock(1);                      // Execute the T1 cycle of M1 (Opcode Fetch machine cycle)
  digitalWrite(RAM_CE2, LOW);         // Force the RAM in HiZ (CE2 = LOW)
  DDRA = 0xFF;                        // Configure Z80 data bus D0-D7 (PA0-PA7) as output
  PORTA = LD_HLnn;                    // Write "LD HL, n" opcode on data bus
  pulseClock(2);                      // Execute T2 and T3 cycles of M1
  DDRA = 0x00;                        // Configure Z80 data bus D0-D7 (PA0-PA7) as input... 
  PORTA = 0xFF;                       // ...with pull-up
  pulseClock(2);                      // Complete the execution of M1 and execute the T1 cycle of the following 
                                      // Memory Read machine cycle
  DDRA = 0xFF;                        // Configure Z80 data bus D0-D7 (PA0-PA7) as output
  PORTA = lowByte(value);             // Write first byte of "value" to load in HL
  pulseClock(3);                      // Execute the T2 and T3 cycles of the first Memory Read machine cycle
                                      // and T1, of the second Memory Read machine cycle
  PORTA = highByte(value);            // Write second byte of "value" to load in HL
  pulseClock(2);                      // Execute the T2 and T3 cycles of the second Memory Read machine cycle
  DDRA = 0x00;                        // Configure Z80 data bus D0-D7 (PA0-PA7) as input... 
  PORTA = 0xFF;                       // ...with pull-up
  digitalWrite(RAM_CE2, HIGH);        // Enable the RAM again (CE2 = HIGH)
}

// ------------------------------------------------------------------------------

void singlePulsesResetZ80()
// Reset the Z80 CPU using single pulses clock
{
  digitalWrite(RESET_, LOW);          // Set RESET_ active
  pulseClock(6);                      // Generate twice the needed clock pulses to reset the Z80
  digitalWrite(RESET_, HIGH);         // Set RESET_ not active
  pulseClock(2);                      // Needed two more clock pulses after RESET_ goes HIGH
}

void bootZ80() {
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
    case 3:                                       // EEPROM scratchpad boot
      // we can't just set BootImage because its expecting a region in PROGMEM
      BootImageSize = 0xFF;
      BootStrAddr = 0x0000;
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
    if( bootMode != 3) 
      loadByteToRAM(pgm_read_byte(BootImage + i));  // Write current data byte into RAM
    else
      loadByteToRAM(EEPROM.read(nvStorageAddr + i));
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

