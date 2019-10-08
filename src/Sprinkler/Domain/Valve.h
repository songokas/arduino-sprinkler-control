#ifndef SPRINKLER_DOMAIN_VALVE_H
#define SPRINKLER_DOMAIN_VALVE_H

#include <Arduino.h>

namespace Sprinkler
{
    namespace Domain
    {
        class Valve
        {
        public:
            Valve(uint8_t controlPin);
            bool open();
            bool close();
        private:
            uint8_t controlPin { 0 };
        };
    }
}

#endif
