#ifndef SPRINKLER_DOMAIN_SOIL_SENSOR_H
#define SPRINKLER_DOMAIN_SOIL_SENSOR_H

#include <Arduino.h>

#ifndef SOIL_SENSOR_SAMPLE_SIZE       
#define SOIL_SENSOR_SAMPLE_SIZE 5
#endif


namespace Sprinkler
{
    namespace Domain
    {
        class SoilSensor
        {
            public:
                SoilSensor(uint8_t readPin, uint16_t wetTo = 290, uint16_t dryFrom = 350, unsigned long readInterval = 10000);
                uint16_t read();
                bool isWet();
                bool isDry();
            private:
                uint16_t smoothValues();
                uint8_t readPin { 0 };
                uint16_t wetTo { 0 };
                uint16_t dryFrom { 0 };
                unsigned long lastRead {0};
                unsigned long readInterval {10000};
                uint16_t valuesRead[SOIL_SENSOR_SAMPLE_SIZE] {0};
                uint8_t valueIndex {0};
        };
    }
}

#endif
