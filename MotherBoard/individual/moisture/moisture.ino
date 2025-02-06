#include <Arduino.h>

// Define the sensor pins
const int sensorPins[4] = {4, 5, 6, 7};
int sensorValues[4] = {0, 0, 0, 0}; // Array to store sensor readings

// Function to read values from capacitive sensors and print them
void readAndPrintCapacitiveSensors() {
  Serial.println("-----------------------");
    for (int i = 0; i < 4; i++) {
        sensorValues[i] = analogRead(sensorPins[i]);
        Serial.print("Sensor ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(sensorValues[i]);
    }
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);

    // Set the sensor pins as input
    for (int i = 0; i < 4; i++) {
        pinMode(sensorPins[i], INPUT);
    }
}

void loop() {
    // Read and print sensor values
    readAndPrintCapacitiveSensors();

    // Add a delay to avoid flooding the serial monitor
    delay(1000); // Wait 1 second before the next reading
}
