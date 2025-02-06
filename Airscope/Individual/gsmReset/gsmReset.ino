#include <Arduino.h>

//const int gpioPin = 26; //GSM Relay reset
const int gpioPin = 13; //Fan control

void setup() {
    Serial.begin(115200); // Start serial communication at 115200 baud rate
    pinMode(gpioPin, OUTPUT); // Set GPIO 3 as an output
    digitalWrite(gpioPin, LOW); // Ensure the pin is initially off
}

void loop() {
    // Check if data is available to read
    if (Serial.available() > 0) {
        // Read the input
        int state = Serial.parseInt();

        // Turn the GPIO pin on or off based on the input
        if (state == 1) {
            digitalWrite(gpioPin, HIGH); // Turn GPIO 3 on
            Serial.println("GPIO 3 is ON");
        } else if (state == 0) {
            digitalWrite(gpioPin, LOW); // Turn GPIO 3 off
            Serial.println("GPIO 3 is OFF");
        } else {
            Serial.println("Invalid input. Enter 1 to turn on and 0 to turn off.");
        }
    }
}
