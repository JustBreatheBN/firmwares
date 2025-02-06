#include <Arduino.h>

void setup() {
  Serial.begin(115200);         // Initialize serial communication for debugging
  Serial2.begin(9600, SERIAL_8N1,16,17); // Initialize Serial2 for HC-12 communication

  delay(1000); // Give some time to initialize
  Serial.println("ESP32-S3 Device 1");
}

void loop() {
  // Send message to HC-12
  Serial2.println("Hello from ESP32-S3 Device 1");

  // Print to Serial Monitor for debugging
  Serial.println("Sent: Hello from ESP32-S3 Device 1");

  // Check if there is data available to read from HC-12
  if (Serial2.available() > 0) {
    // Read the message from HC-12
    String message = Serial2.readStringUntil('\n');

    // Print the received message to Serial Monitor
    Serial.print("Received: ");
    Serial.println(message);
  }

  // Wait for 1 second before sending the next message
  delay(1000);
}
