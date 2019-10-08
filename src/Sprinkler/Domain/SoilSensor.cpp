#include "CommonModule/MacroHelper.h"
#include "SoilSensor.h"

using Sprinkler::Domain::SoilSensor;

SoilSensor::SoilSensor(uint8_t readPin, uint16_t wetTo, uint16_t dryFrom, unsigned long readInterval)
  : readPin(readPin), wetTo(wetTo), dryFrom(dryFrom), readInterval(readInterval)
{
}

uint16_t SoilSensor::read()
{
    return analogRead(readPin);
  if (!lastRead) {
    valuesRead[valueIndex] = analogRead(readPin);
    valueIndex++;
  } else if (millis() - lastRead > readInterval) {
    valuesRead[valueIndex] = analogRead(readPin);
    lastRead = millis();
    valueIndex++;
  }
  if (valueIndex >= COUNT_OF(valuesRead)) {
    valueIndex = 0;
  }
  return smoothValues();
}

uint16_t SoilSensor::smoothValues()
{
    uint8_t count {0};
    uint32_t total {0};
    for (auto val: valuesRead) {
        if (val == 0) {
            continue;
        }
        total += val;
        count++;
    }
    return count > 0 ? total / count : 0;
}

bool SoilSensor::isWet()
{
  return read() < wetTo;
}

bool SoilSensor::isDry()
{
  return read() > dryFrom;
}
