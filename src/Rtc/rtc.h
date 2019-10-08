#ifndef RTC_RTC_H
#define RTC_RTC_H

#include <Wire.h> // must be included here so that Arduino library object file references work
#include <RtcDS3231.h>

void printDateTime(const RtcDateTime& dt);
void rtcSetup(RtcDS3231<TwoWire> & Rtc);
//void rtcCheck(RtcDS3231<TwoWire> & Rtc);

#endif
