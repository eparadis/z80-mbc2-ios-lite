#ifndef _PINS_H
#define _PINS_H

// ------------------------------------------------------------------------------
//
// Hardware definitions for A040618 (Z80-MBC2) - Base system
//
// ------------------------------------------------------------------------------

#define   D0          24  // PA0 pin 40   Z80 data bus
#define   D1          25  // PA1 pin 39
#define   D2          26  // PA2 pin 38
#define   D3          27  // PA3 pin 37
#define   D4          28  // PA4 pin 36
#define   D5          29  // PA5 pin 35
#define   D6          30  // PA6 pin 34
#define   D7          31  // PA7 pin 33

#define   AD0         18  // PC2 pin 24   Z80 A0
#define   WR_         19  // PC3 pin 25   Z80 WR
#define   RD_         20  // PC4 pin 26   Z80 RD
#define   MREQ_       21  // PC5 pin 27   Z80 MREQ
#define   RESET_      22  // PC6 pin 28   Z80 RESET
#define   MCU_RTS_    23  // PC7 pin 29   * RESERVED - NOT USED *
#define   MCU_CTS_    10  // PD2 pin 16   * RESERVED - NOT USED *
#define   BANK1       11  // PD3 pin 17   RAM Memory bank address (High)
#define   BANK0       12  // PD4 pin 18   RAM Memory bank address (Low)
#define   INT_         1  // PB1 pin 2    Z80 control bus
#define   RAM_CE2      2  // PB2 pin 3    RAM Chip Enable (CE2). Active HIGH. Used only during boot
#define   WAIT_        3  // PB3 pin 4    Z80 WAIT
#define   SS_          4  // PB4 pin 5    SD SPI * RESERVED - NOT USED *
#define   MOSI         5  // PB5 pin 6    SD SPI * RESERVED - NOT USED *
#define   MISO         6  // PB6 pin 7    SD SPI * RESERVED - NOT USED *
#define   SCK          7  // PB7 pin 8    SD SPI * RESERVED - NOT USED *
#define   BUSREQ_     14  // PD6 pin 20   Z80 BUSRQ
#define   CLK         15  // PD7 pin 21   Z80 CLK
#define   SCL_PC0     16  // PC0 pin 22   IOEXP connector (I2C)
#define   SDA_PC1     17  // PC1 pin 23   IOEXP connector (I2C)
#define   LED_IOS      0  // PB0 pin 1    Led LED_IOS is ON if HIGH
#define   WAIT_RES_    0  // PB0 pin 1    Reset the Wait FF
#define   USER        13  // PD5 pin 19   Led USER and key (led USER is ON if LOW)

#endif
