#include <RtcDS3231.h>
#include "Timer.h"

using Sprinkler::Domain::Timer;

Timer::Timer(uint8_t fromHour, uint8_t fromMinute, uint8_t toHour, uint8_t toMinute)
  :fromHour(fromHour), fromMinute(fromMinute), toHour(toHour), toMinute(toMinute)
{}

bool Timer::isEmpty() const
{
    return fromHour == 0 && fromMinute == 0 && toHour == 0 && toMinute == 0;
}

bool Timer::isOn(const RtcDateTime & dt) const
{
    if (isEmpty() || !dt.IsValid()) {
        return false;
    }
    if (dt.Hour() >= fromHour && dt.Hour() <= toHour) {
        return dt.Minute() >= fromMinute && dt.Minute() < toMinute;
    }
    return false;
}
