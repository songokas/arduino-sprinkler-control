#include "Valve.h"

using Sprinkler::Domain::Valve;

Valve::Valve(uint8_t controlPin): controlPin(controlPin)
{
  pinMode(controlPin, OUTPUT);
  close();
}

bool Valve::open()
{
  digitalWrite(controlPin, LOW);
  return true;
}

bool Valve::close()
{
  digitalWrite(controlPin, HIGH);
  return true;
}
