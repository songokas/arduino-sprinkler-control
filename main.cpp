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


/*
submodule "libs/arduinolibs"]
        path = libs/arduinolibs
        url = https://github.com/rweather/arduinolibs
[submodule "libs/Entropy"]
        path = libs/Entropy
        url = https://github.com/pmjdebruijn/Arduino-Entropy-Library
[submodule "libs/RF24"]
        path = libs/RF24
        url = https://github.com/nRF24/RF24
[submodule "libs/RF24Network"]
        path = libs/RF24Network
        url = https://github.com/nRF24/RF24Network
[submodule "libs/RF24Mesh"]
        path = libs/RF24Mesh
        url = https://github.com/nRF24/RF24Mesh
[submodule "libs/Rtc"]
        path = libs/Rtc
        url = https://github.com/Makuna/Rtc.git

        path = libs/Catch2
        url = https://github.com/catchorg/Catch2
[submodule "libs/Array"]
        path = libs/Array
        url = https://github.com/janelia-arduino/Array
[submodule "libs/Streaming"]
        path = libs/Streaming
        url = https://github.com/janelia-arduino/Streaming
[submodule "libs/Vector"]
        path = libs/Vector
        url = https://github.com/janelia-arduino/Vector
[submodule "libs/FakeIt"]
        path = libs/FakeIt
        url = https://github.com/eranpeer/FakeIt
[submodule "libs/esp8266"]
        path = libs/esp8266
        url = https://github.com/esp8266/Arduino.git
[submodule "libs/MemoryFree"]
        path = libs/MemoryFree
        url = https://github.com/McNeight/MemoryFree
[submodule "libs/pubsubclient"]
        path = libs/pubsubclient
        url = https://github.com/knolleary/pubsubclient
[submodule "libs/ESP8266TrueRandom"]
        path = libs/ESP8266TrueRandom
        url = https://github.com/marvinroger/ESP8266TrueRandom
[submodule "libs/CRC32"]
        path = libs/CRC32
        url = https://github.com/bakercp/CRC32
[submodule "libs/ArduinoJson"]
        path = libs/ArduinoJson
        url = https://github.com/bblanchon/ArduinoJson
[submodule "libs/DallasTemperature"]
        path = libs/DallasTemperature
        url = https://github.com/milesburton/Arduino-Temperature-Control-Library
[submodule "libs/OneWire"]
        path = libs/OneWire
        url = https://github.com/PaulStoffregen/OneWire
*/

/*
git submodule add https://github.com/rweather/arduinolibs libs/arduino-libs
git submodule add https://github.com/pmjdebruijn/Arduino-Entropy-Library libs/Entropy
git submodule add https://github.com/bakercp/CRC32 libs/CRC32
git submodule add https://github.com/janelia-arduino/Streaming libs/Streaming
git submodule add https://github.com/Makuna/Rtc.git libs/Rtc
git submodule add https://github.com/nRF24/RF24 libs/RF24
git submodule add https://github.com/nRF24/RF24Network libs/RF24Network
git submodule add https://github.com/nRF24/RF24Mesh libs/RF24Mesh
git submodule add https://github.com/songokas/arduino-tranceiver-encrypted libs/arduino-tranceiver-encrypted 
git submodule add https://github.com/McNeight/MemoryFree libs/MemoryFree
git submodule add 
git submodule add 
git submodule add 
git submodule add 
git submodule add https://github.com/rweather/arduinolibs libs/arduino-libs
git submodule add https://github.com/rweather/arduinolibs libs/arduino-libs
git submodule add https://github.com/rweather/arduinolibs libs/arduino-libs
git submodule add https://github.com/rweather/arduinolibs libs/arduino-libs
git submodule add https://github.com/rweather/arduinolibs libs/arduino-libs
git submodule add https://github.com/rweather/arduinolibs libs/arduino-libs
*/
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

void operator delete(void* obj, unsigned int n) { 
    free(obj); 
} 

bool reconnect(RF24Mesh & mesh)
{
    if (!mesh.checkConnection() ) {
        Serial << F("Renewing Mesh Address") << endl;
        if(!mesh.renewAddress(MESH_TIMEOUT)){
            return mesh.begin(RADIO_CHANNEL, RF24_250KBPS, MESH_TIMEOUT);
        } else {
            return true;
        }
    } else {
        auto currentAddress = mesh.getAddress(mesh.getNodeID());
        if (!(currentAddress > 0)) {
            Serial << F("Renew address: ") << currentAddress << endl;
            return mesh.renewAddress(MESH_TIMEOUT);
        }
    }
    return false;
}

void subscribeToChannel(MeshMqttClient & client, const char * zoneName, IMessageHandler & handler)
{
    char topic[MAX_LEN_TOPIC] {0};
    snprintf_P(topic, COUNT_OF(topic), TOPIC_STATE_CHANGE, zoneName);
    if (!client.subscribe(topic, &handler)) {
        Serial << (F("Failed to subscribe")) << endl;
    }
    wdt_reset();
}

void sendStateData(MeshMqttClient & client, const char * zoneName, bool state)
{
    char topic[MAX_LEN_TOPIC] {0};
    snprintf_P(topic, COUNT_OF(topic), TOTIC_STATE, zoneName);

    if (!client.publish(topic, (uint16_t)state)) {
        Serial << (F("Failed to publish state")) << endl;
    }
    wdt_reset();
}

void sendSoilData(MeshMqttClient & client, const char * zoneName, uint16_t sensor)
{
    char topic[MAX_LEN_TOPIC] {0};
    snprintf_P(topic, COUNT_OF(topic), TOTIC_SOIL, zoneName);

    if (!client.publish(topic, sensor)) {
        Serial << (F("Failed to publish sensor")) << endl;
    }
    wdt_reset();
}

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
