#include <Arduino.h>
#include <Streaming.h>

#include "CommonModule/MacroHelper.h"
#include "rtc.h"

void printDateTime(const RtcDateTime& dt)
{
    char datestring[20];

    snprintf_P(datestring, 
            COUNT_OF(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            dt.Month(),
            dt.Day(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    Serial << datestring << endl;
}

void rtcSetup(RtcDS3231<TwoWire> & Rtc)
{
    Rtc.Begin();

    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
    printDateTime(compiled);

    if (!Rtc.IsDateTimeValid()) {
        if (Rtc.LastError() != 0) {
            // we have a communications error
            // see https://www.arduino.cc/en/Reference/WireEndTransmission for 
            // what the number means
            DPRINT(F("RTC communications error = "));
            DPRINTLN(Rtc.LastError());
        } else {
            // Common Causes:
            //    1) first time you ran and the device wasn't running yet
            //    2) the battery on the device is low or even missing

            DPRINTLN(F("RTC lost confidence in the DateTime!"));

            // following line sets the RTC to the date & time this sketch was compiled
            // it will also reset the valid flag internally unless the Rtc device is
            // having an issue

            Rtc.SetDateTime(compiled);
        }
    }

    if (!Rtc.GetIsRunning()) {
        DPRINTLN(F("RTC starting"));
        Rtc.SetIsRunning(true);
    }

    RtcDateTime now = Rtc.GetDateTime();
    if (now < compiled) {
        DPRINTLN(F("RTC update with compile time"));
        Rtc.SetDateTime(compiled);
    }
    /*else if (now > compiled) {
        DPRINTLN(F("RTC is newer than compile time. (this is expected)"));
    }
    else if (now == compiled) {
        DPRINTLN(F("RTC is the same as compile time! (not expected but all is fine)"));
    }*/

    // never assume the Rtc was last configured by you, so
    // just clear them to your needed state
    Rtc.Enable32kHzPin(false);
    Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone); 
}


/*void rtcCheck(RtcDS3231<TwoWire> & Rtc)
{
    if (!Rtc.IsDateTimeValid()) 
    {
        if (Rtc.LastError() != 0)
        {
            // we have a communications error
            // see https://www.arduino.cc/en/Reference/WireEndTransmission for 
            // what the number means
            DPRINT(F("RTC communications error = "));
            DPRINTLN(Rtc.LastError());
        }
        else
        {
            // Common Causes:
            //    1) the battery on the device is low or even missing and the power line was disconnected
            DPRINTLN(F("RTC lost confidence in the DateTime!"));
        }
    }
}*/
