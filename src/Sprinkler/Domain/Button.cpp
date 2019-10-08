#include "Button.h"

using Sprinkler::Domain::Button;

Button::Button(uint8_t controlPin/*, uint16_t debounceDelay*/): controlPin(controlPin)//, debounceDelay(debounceDelay)
{
    if (controlPin) {
        pinMode(controlPin, INPUT);
    }
}

bool Button::isPressed() 
{
    if (!(controlPin > 0)) {
        return false;
    }
    int buttonState = digitalRead(controlPin);
    //Serial.println(buttonState);
    return buttonState == HIGH;

    /*if (lastButtonState != buttonState) {
        lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) > debounceDelay) {
        lastButtonState = buttonState;
        return buttonState == HIGH;
    }
    return false;*/
}
