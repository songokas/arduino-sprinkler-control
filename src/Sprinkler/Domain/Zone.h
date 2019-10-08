#ifndef SPRINKLER_DOMAIN_ZONE_H
#define SPRINKLER_DOMAIN_ZONE_H

#include <Arduino.h>
//#include <Wire.h>
//#include <RtcDS3231.h>

#include "SoilSensor.h"
#include "Valve.h"
#include "Button.h"
#include "Timer.h"
#include "State.h"

namespace Sprinkler
{
    namespace Domain
    {
        class Zone
        {
            public:

                Zone(const char * name, const Timers & alarms, const Button & button, const SoilSensor & soilSensor, const Valve & valve, uint16_t manualChangeTimeout = 1800);
                State getExpectedState(const RtcDateTime & dt);
                const char * getName() const;
                bool shouldToggle(const State & state) const;
                void toggle(const State & state);
                const State & getCurrentState() const;
                uint16_t getSoilSensorValue();
                bool isOn();
            private:
                const char * name;
                const Timers alarms;
                Button button;
                SoilSensor soilSensor;
                Valve valve;
                State currentState {StateType::ALARM_OFF, RtcDateTime(0)};
                uint16_t manualChangeTimeout {1800};
        };
    }
}

#endif
