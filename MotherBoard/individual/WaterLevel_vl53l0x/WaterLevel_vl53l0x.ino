#include <Wire.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
    Serial.begin(115200);

    // Wait until serial port opens for native USB devices
    while (!Serial) {
        delay(1);
    }

    Serial.println("Adafruit VL53L0X test");

    // Initialize the I2C bus
    Wire.begin(1,2);

    // Initialize the VL53L0X sensor
    if (!lox.begin()) {
        Serial.println(F("Failed to boot VL53L0X"));
        while (1);
    }

    // Power
    Serial.println(F("VL53L0X API Simple Ranging example\n\n"));
}

void loop() {
    VL53L0X_RangingMeasurementData_t measure;

    Serial.print("Reading a measurement... ");
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

    if (measure.RangeStatus != 4) {  // Phase failures have incorrect data
        Serial.print("Distance (mm): ");
        Serial.println(measure.RangeMilliMeter);
    } else {
        Serial.println(" out of range ");
    }

    delay(100);
}
