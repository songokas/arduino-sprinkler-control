#ifndef SPRINKLER_DOMAIN_TIMER
#define SPRINKLER_DOMAIN_TIMER

#include <Arduino.h>

#ifndef MAX_TIMERS
#define MAX_TIMERS 2
#endif

class RtcDateTime;

namespace Sprinkler
{
    namespace Domain
    {

        class Timer
        {
            public:
                Timer(uint8_t fromHour, uint8_t fromMinute, uint8_t toHour, uint8_t toMinute);
                bool isOn(const RtcDateTime& dt) const;
                bool isEmpty() const;
            private:
                uint8_t fromHour { 0 };
                uint8_t fromMinute { 0 };
                uint8_t toHour { 0 };
                uint8_t toMinute { 0 };
        };    

        struct Timers
        {
            Timer values[MAX_TIMERS];
        };
    }
};

#endif
