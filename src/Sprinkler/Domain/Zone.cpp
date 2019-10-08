#include <Wire.h>
#include <RtcDS3231.h>

#include "CommonModule/MacroHelper.h"
#include "Zone.h"

using Sprinkler::Domain::Zone;
using Sprinkler::Domain::SoilSensor;
using Sprinkler::Domain::Valve;
using Sprinkler::Domain::Button;
using Sprinkler::Domain::Timer;
using Sprinkler::Domain::State;


Zone::Zone(const char * name, const Timers & alarms, const Button & button, const SoilSensor & soilSensor, const Valve & valve, uint16_t manualChangeTimeout)
  :name(name), alarms(alarms), button(button), soilSensor(soilSensor), valve(valve), manualChangeTimeout(manualChangeTimeout)
{}

State Zone::getExpectedState(const RtcDateTime & dt)
{
  if (button.isPressed()) {
      if (currentState.isOff()) {
        return {StateType::BUTTON_ON, dt};
      } else  {
        return {StateType::BUTTON_OFF, dt};
      }
  } else if (currentState.isManualFresh(dt, manualChangeTimeout)) {
    return currentState;
  }

  bool alarmOn = false;
  for (uint8_t index = 0; index < COUNT_OF(alarms.values); index++) {
    if (alarms.values[index].isOn(dt)) {
      alarmOn = true;
      break;
    }
  }

  if (alarmOn) {
    if (currentState.isOnByAlarm() && soilSensor.isDry()) {
        return currentState;
    }
    else if (soilSensor.isDry()) {
        return State {StateType::ALARM_ON, dt};
    }
  }
  return State {StateType::ALARM_OFF, dt};
}

const char * Zone::getName() const
{
  return name;
}

const State & Zone::getCurrentState() const
{
    return currentState;
}

bool Zone::shouldToggle(const State & state) const
{
    if (state.getType() != currentState.getType()) {
        return true;    
    }
    return false;
}

void Zone::toggle(const State & state)
{
    if (state.isOff()) {
      valve.close();
    } else {
      valve.open();
    }
    currentState = state;
    //memcpy(&currentState, &state, sizeof(state));
}

uint16_t Zone::getSoilSensorValue()
{
    return soilSensor.read();
}

bool Zone::isOn()
{
    return currentState.isOn();
}
