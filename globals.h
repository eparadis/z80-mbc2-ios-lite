#ifndef _GLOBALS_H
#define _GLOBALS_H

void blinkIOSled(unsigned long *timestamp);

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

#define bootModeAddr  10          // Internal EEPROM address for boot mode storage
#define autoexecFlagAddr 12       // Internal EEPROM address for AUTOEXEC flag storage
#define clockModeAddr 13          // Internal EEPROM address for the Z80 clock 4/8 MHz switch
#define nvStorageAddr 0x0100      // Internal EEPROM address for non-volatile storage (256 bytes total)

#define debug 0        // Debug off = 0, on = 1


#endif
