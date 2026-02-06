#include <Arduino.h>
#include <SPI.h>

#include "adxl375_rp.h"

#define VMOSI   15
#define VMISO   16
#define VCLK    17
#define SPI2_CS 9

SPIClass spi;
ADXL375_RP adxl(spi, SPI2_CS, ADXL375_RP::ADXL375_RATE_1600HZ);

ADXL375_RP_Reading readings[33];

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("hello world");

    spi.begin(VCLK, VMISO, VMOSI, -1);

    if (!adxl.begin()) {
        Serial.println("bruh");
        while (1)
            ;
    } else {
        Serial.println("Startup worked!");
    }
}

void loop() {
    size_t num_readings = adxl.read(readings);

    Serial.printf("Got %lu readings\n", num_readings);

    for (int i = 0; i < num_readings; i++) {
        Serial.printf(
            "%d: %f,%f,%f, %lu\n",
            i,
            readings[i].x,
            readings[i].y,
            readings[i].z,
            readings[i].timestamp
        );
    }

    Serial.println();

    delay(10);
}