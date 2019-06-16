#ifndef _RTC_H
#define _RTC_H

byte autoSetRTC();
void ChangeRTC();
void readRTC(byte *second, byte *minute, byte *hour, byte *day, byte *month, byte *year, byte *tempC);

#endif
