#include <Arduino.h>

// Define the door latch sensor pin
const int doorLatchSensorPin = 45;
int doorLatchLastState = LOW; // Variable to store the last state of the door latch sensor
int doorLatchCurrentState = LOW; // Variable to store the current state of the door latch sensor

void setup() {
    // Initialize serial communication
    Serial.begin(115200);

    // Initialize the door latch sensor pin as input
    pinMode(doorLatchSensorPin, INPUT);

    // Read the initial state of the door latch sensor
    doorLatchLastState = digitalRead(doorLatchSensorPin);
}

void loop() {
    // Read the current state of the door latch sensor
    doorLatchCurrentState = digitalRead(doorLatchSensorPin);

    // Check if the state has changed
    if (doorLatchCurrentState != doorLatchLastState) {
        // State has changed
        if (doorLatchCurrentState == HIGH) {
            // Sensor changed from low to high (closed to open)
            Serial.println("Door opened");
        } else {
            // Sensor changed from high to low (open to closed)
            Serial.println("Door closed");
        }

        // Update the last state
        doorLatchLastState = doorLatchCurrentState;
    }

    // Small delay to debounce the sensor
    delay(50);
}
