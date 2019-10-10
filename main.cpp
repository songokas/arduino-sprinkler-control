#include <Arduino.h>
#include <avr/wdt.h>
#include <RF24.h>
#include <RF24Network.h>
#include <RF24Mesh.h>
#include <Crypto.h>
#include <CryptoLW.h>
#include <Acorn128.h>
#include <Entropy.h>
#include <Streaming.h>
// make .mk compiler happy
#include <Wire.h>
#include <RtcDS3231.h>
#include <AuthenticatedCipher.h>
#include <Cipher.h>
#include <SPI.h>
#include <CRC32.h>
//
#include <MemoryFree.h>

#include "CommonModule/MacroHelper.h"
#include "RadioEncrypted/RadioEncryptedConfig.h"
#include "Sprinkler/Domain/Zone.h"
#include "Rtc/rtc.h"
#include "RadioEncrypted/Encryption.h"
#include "RadioEncrypted/EncryptedMesh.h"
#include "MqttModule/MeshMqttClient.h"
#include "MqttModule/SubscriberList.h"
#include "RadioEncrypted/Entropy/AvrEntropyAdapter.h"

#include "ZoneMessageHandlers/ZoneMessageHandler.h"

using Sprinkler::Domain::Zone;
using Sprinkler::Domain::SoilSensor;
using Sprinkler::Domain::Valve;
using Sprinkler::Domain::Button;
using Sprinkler::Domain::Timer;
using Sprinkler::Domain::State;
using Sprinkler::Domain::StateType;

using RadioEncrypted::Encryption;
using RadioEncrypted::Entropy::AvrEntropyAdapter;
using RadioEncrypted::EncryptedMesh;

using MqttModule::SubscriberList;
using MqttModule::MeshMqttClient;

using ZoneMessageHandlers::ZoneMessageHandler;

const uint16_t ZONE_TOGGLE_DELAY {2000};
const uint16_t DISPLAY_TIME { 30000 };

const uint16_t BAUD_RATE {9600};

const uint8_t CE_PIN = 7;
const uint8_t CN_PIN = 8;

// no more than TOPIC_MAX_LENGTH
const char TOPIC_STATE_CHANGE [] PROGMEM {"laistymas/%s/set/jungtukas"};
const char TOTIC_STATE[] PROGMEM {"laistymas/%s/jungtukas"};
const char TOTIC_SOIL[] PROGMEM {"laistymas/%s/soilSensor"};

#include "src/helpers.h"

int main()
{

  init();

  Serial.begin(BAUD_RATE);

  Serial << F("freeMemory ") << freeMemory() << endl;

  wdt_enable(WDTO_8S);

  RF24 radio(CE_PIN, CN_PIN);
  RF24Network network(radio);
  RF24Mesh mesh(radio, network);

  Acorn128 cipher;
  EntropyClass entropy;
  entropy.initialize();
  AvrEntropyAdapter entropyAdapter(entropy);
  Encryption encryption (cipher, SHARED_KEY, entropyAdapter);
  EncryptedMesh encMesh (mesh, network, encryption);

  mesh.setNodeID(NODE_ID);
  // Connect to the mesh
  Serial << F("Connecting to the mesh...") << endl;
  if (!mesh.begin(RADIO_CHANNEL, RF24_250KBPS, MESH_TIMEOUT)) {
    Serial << F("Failed to connect to mesh") << endl;
  }

  wdt_reset();

  RtcDS3231<TwoWire> rtc(Wire);
  Serial << F("compiled:") << __DATE__ << __TIME__ << endl;
  rtcSetup(rtc);

  wdt_reset();

  Zone zones[] = { 
    {"pagrindine", {{ {7, 0, 8, 0}, {19, 0, 19, 40}}}, Button(0), SoilSensor(A0), Valve(9)},
    {"uz-namo",  {{ {8, 0, 8, 49}, {19, 40, 20, 20}}}, Button(0), SoilSensor(A1), Valve(6)},
    {"kapiliarine", {{ {9, 0, 10, 0}, {20, 20, 21, 0}}}, Button(0), SoilSensor(A2), Valve(2)},
  };

  ZoneMessageHandler handlers[] = {
      {zones[0], rtc},
      {zones[1], rtc},
      {zones[2], rtc},
  };

  SubscriberList subscribers;
  MeshMqttClient client(encMesh, subscribers);

  unsigned long lastRefreshTime = 0;
  bool printToScreen = true;
  bool subscribeToChannels = true;

  while (true) {

    //rtcCheck(rtc);
    mesh.update();
    client.loop();

    uint8_t index = 0;
    for (auto & zone: zones) {
        
        RtcDateTime now = rtc.GetDateTime();
        State state = zone.getExpectedState(now);
        if (zone.shouldToggle(state)) {
            zone.toggle(state);
            delay(ZONE_TOGGLE_DELAY);
            wdt_reset();
            sendStateData(client, zone.getName(), zone.isOn());
        }

        mesh.update();
        client.loop();

	    if (printToScreen) {
            State currentState = zone.getCurrentState();

            Serial << F("Zone: ") << zone.getName() << F(" is ") << (currentState.isOn() ? F("ON") : F("OFF"))
                << F(" ") << F(" soil: ") << zone.getSoilSensorValue()
                << F(" free memory: ") << freeMemory() << endl;

            sendStateData(client, zone.getName(), zone.isOn());

            mesh.update();
            client.loop();

            sendSoilData(client, zone.getName(), zone.getSoilSensorValue());
	    }

	    if (subscribeToChannels)  {
            subscribeToChannel(client, zone.getName(), handlers[index]);
	    }

        wdt_reset();
        mesh.update();
        client.loop();
        index++;
        //delay(3000);
    }

    if (printToScreen) {
        printToScreen = false;
    }
    if (subscribeToChannels) {
        subscribeToChannels = false;
    }

	
	if(millis() - lastRefreshTime >= DISPLAY_TIME) {
        printDateTime(rtc.GetDateTime());
		lastRefreshTime += DISPLAY_TIME;
		printToScreen = true;

		if (reconnect(mesh)) {
		    subscribeToChannels = true;
        }
	}
    //Serial << F("Ping") << endl;
    wdt_reset();

  }
  
}
