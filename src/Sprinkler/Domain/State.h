#ifndef SPRINKLER_DOMAIN_STATE
#define SPRINKLER_DOMAIN_STATE

#include <Arduino.h>
#include <Wire.h>
#include <RtcDS3231.h>

namespace Sprinkler
{
    namespace Domain
    {
        enum class StateType
        {
            BUTTON_ON,
            ALARM_ON,
            ALARM_OFF,
            BUTTON_OFF,
            CONTROL_ON,
            CONTROL_OFF
        };

        class State
        {
            public:
                State(const StateType & type, const RtcDateTime & dt);
                bool isOff() const;
                bool isOn() const;
                bool isManualFresh(const RtcDateTime & dt, uint16_t manualChangeTimeout) const;
                bool isOnByButton() const;
                bool isOnByAlarm() const;
                const StateType & getType() const;
                bool isManual() const;
            private:
                StateType type { StateType::ALARM_OFF };
                RtcDateTime dtc { 0 };
        };
    }
}

#endif
