#ifndef RADIO_ENCRYPTED_MQTT_MESSAGE_HANDLER_H
#define RADIO_ENCRYPTED_MQTT_MESSAGE_HANDLER_H

#include <Wire.h>
#include <RtcDS3231.h>
#include "MqttModule/MessageHandlers/IMessageHandler.h"

namespace Sprinkler
{
    namespace Domain
    {
        class Zone;
    }
}

using Sprinkler::Domain::Zone;
using MqttModule::MessageHandlers::IMessageHandler;

namespace ZoneMessageHandlers
{
    class ZoneMessageHandler: public IMessageHandler
    {
        public:
            ZoneMessageHandler(Zone & zone, RtcDS3231<TwoWire> & rtc);
            void handle(const char * channel, const char * message);
        private:
            Zone & zone;
            RtcDS3231<TwoWire> & rtc;
    };
}

#endif
