#include "CommonModule/MacroHelper.h"
#include "Sprinkler/Domain/State.h"
#include "Sprinkler/Domain/Zone.h"
#include "ZoneMessageHandler.h"
#include "MqttModule/MqttMessage.h"

using Sprinkler::Domain::StateType;
using Sprinkler::Domain::Zone;
using Sprinkler::Domain::State;
using ZoneMessageHandlers::ZoneMessageHandler;

ZoneMessageHandler::ZoneMessageHandler(Zone & zone, RtcDS3231<TwoWire> & rtc)
    :zone(zone), rtc(rtc)
{}

void ZoneMessageHandler::handle(const char * topic, const char * message)
{
    StateType type = strcmp(message, "1") == 0 ? StateType::CONTROL_ON : StateType::CONTROL_OFF;
    DPRINT(F("Mqtt handler called with message: ")); DPRINTLN(message);
    zone.toggle(State(type, rtc.GetDateTime()));
}
