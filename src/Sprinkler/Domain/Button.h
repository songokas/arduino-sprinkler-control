#ifndef SPRINKLER_DOMAIN_BUTTON_H
#define SPRINKLER_DOMAIN_BUTTON_H

#include <Arduino.h>

namespace Sprinkler
{
    namespace Domain
    {

        class Button
        {
            public:
                Button(uint8_t controlPin/*, uint16_t debounceDelay = 100*/);
                bool isPressed();
            private:
                uint8_t controlPin { 0 };
                //uint32_t debounceDelay { 100 };
                //uint32_t lastDebounceTime { 0 };
                //uint8_t lastButtonState { LOW };
        };
    }
}

#endif
