#include <Arduino.h>

// Define the GPIO pins for the relays
const int relayPins[] = {25, 26, 27, 14, 12, 13, 15, 4, 2};

// Number of relays
const int numRelays = sizeof(relayPins) / sizeof(relayPins[0]);

// Function to initialize relay pins
void initializeRelays() {
    for (int i = 0; i < numRelays; i++) {
        pinMode(relayPins[i], OUTPUT);
        digitalWrite(relayPins[i], LOW); // Ensure all relays are off initially
    }
}

// Function to control the relay
void controlRelay(int relayNumber, int state) {
    if (relayNumber >= 1 && relayNumber <= numRelays) {
        int gpio = relayPins[relayNumber - 1];
        digitalWrite(gpio, state);
        Serial.print("Relay ");
        Serial.print(relayNumber);
        Serial.print(" on GPIO ");
        Serial.print(gpio);
        Serial.print(" is turned ");
        Serial.println(state == HIGH ? "ON" : "OFF");
    } else {
        Serial.print("Invalid relay number: ");
        Serial.println(relayNumber);
    }
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);

    // Initialize relay pins
    initializeRelays();

    Serial.println("Enter command in format: <Relay Number> <STATE>");
    Serial.println("Example: 1 1 (to turn on the relay 1)");
    Serial.println("Example: 1 0 (to turn off the relay 1)");
}

void loop() {
    // Check if data is available to read
    if (Serial.available() > 0) {
        // Read the input
        String input = Serial.readStringUntil('\n');

        // Parse the input
        int relayNumber = input.substring(0, input.indexOf(' ')).toInt();
        int state = input.substring(input.indexOf(' ') + 1).toInt();

        // Control the relay based on the input
        controlRelay(relayNumber, state);
    }
}
