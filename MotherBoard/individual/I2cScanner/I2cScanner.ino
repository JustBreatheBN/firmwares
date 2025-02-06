#include <Wire.h>

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial); // Wait for serial port to connect

    // Initialize I2C
//    Wire.begin(1,2);
    Wire.begin();

    Serial.println("I2C Scanner");
}

void loop() {
    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        // The I2C scanner uses the return value of Wire.endTransmission to see if a device did acknowledge to the address
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) 
                Serial.print("0");
            Serial.print(address, HEX);
            Serial.println(" !");
            nDevices++;
        } else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16) 
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0) {
        Serial.println("No I2C devices found\n");
    } else {
        Serial.println("done\n");
    }

    delay(5000); // Wait 5 seconds before scanning again
}
