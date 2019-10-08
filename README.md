# Sprinkler control node

Arduino controls sprinklers based on times set and sensor values received

## Hardware dependencies

* arduino
* nrf24l01
* soil sensors
* control valves
* electric source for valves and arduino
* relays
* time rtc

## Howto

* git clone --recurse-submodules -j8 git://github.com/songokas/arduino-sprinkler-control
* modify source code to change times main.cpp

```
cd arduino-sprinkler-control
cp Makefile-standalone Makefile
# modify Makefile and provide your NODE_ID NODE_NAME etc.
make && make upload && make monitor
```

to control valves with mqtt/phone nrf24l01-mqtt-gateway is needed
