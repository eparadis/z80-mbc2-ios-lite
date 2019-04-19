This is a fork / modification of Just4Fun's excellent firmware for their MBC2 project:

https://hackaday.io/project/159973-z80-mbc2-4ics-homemade-z80-computer

Changes:

- now targetted to an Atmega644 (from an Atmega32A)
    - The '644 has more flash for more storage (64k over 32k)
    - I had one on hand
    - This may also add support for other similar 40-pin Atmegas
- added new IO opcodes that allow EEPROM access
    - a new write opcode that sets the "EEPROM cursor"
    - two more opcodes that read from or write to the current cursor position
    - not the most sophisticated method, but it got things working quickly
    - this scratchpad EEPROM area is 256 bytes long, out of 512 total

Future Changes:

- bulk read and write to EEPROM opcodes
- boot option to copy EEPROM scratch pad to RAM at start-up and then jump execution there
    - this will allow user-definable boot loaders
- I2C connected character devices
    - 40x4 character LCD
    - ps2 keyboard (or 8x8 scanned keyboard)
- a different FORTH that isn't so ornery
