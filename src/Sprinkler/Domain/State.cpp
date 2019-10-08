#include "State.h"

using Sprinkler::Domain::State;
using Sprinkler::Domain::StateType;

State::State (const StateType & type, const RtcDateTime & dt)
    :type(type), dtc(dt)
{
}

bool State::isOff() const
{
    return type == StateType::ALARM_OFF || type == StateType::BUTTON_OFF || type == StateType::CONTROL_OFF;
}

bool State::isOn() const
{
    return !isOff();
}

bool State::isOnByButton() const
{
    return type == StateType::BUTTON_ON;
}

bool State::isOnByAlarm() const
{
    return type == StateType::ALARM_ON;
}

bool State::isManual() const
{
    return type == StateType::BUTTON_ON || type == StateType::BUTTON_OFF || type == StateType::CONTROL_ON || type == StateType::CONTROL_OFF;
}

bool State::isManualFresh(const RtcDateTime & dt, uint16_t manualChangeTimeout) const
{
    return isManual() && dtc.TotalSeconds() + manualChangeTimeout > dt.TotalSeconds();
}

const StateType & State::getType() const
{
    return type;
}


