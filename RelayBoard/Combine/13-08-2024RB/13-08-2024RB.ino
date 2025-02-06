#include "HLW8032.h"
#include <HardwareSerial.h>
#include <Arduino.h>
#include <ArduinoJson.h>

#define SYNC_TIME 60000
HardwareSerial hlSerial(1);  // Use UART1
#define hl_rs 23
#define hl_tx 22
static unsigned long start_time, end_time;
HLW8032 HL;
HardwareSerial MySerial(2);

// Define the GPIO pins for the relays and their feedback
const int relayPins[] = {25, 26, 27, 14, 12, 13, 15, 4, 2};
const int pwmPins[] = {32, 33}; // Declaring the PWM pins

// Number of relays
const int numRelays = sizeof(relayPins) / sizeof(relayPins[0]);

unsigned long lastSentTime = 0;

void initializeRelays() {
  for (int i = 0; i < numRelays; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW); // Ensure all relays are off initially
  }
}

void initializePwmPins() {
  for (int i = 0; i < 2; i++) {
    pinMode(pwmPins[i], OUTPUT);
    analogWrite(pwmPins[i], 0); // Initialize PWM pins to 0
  }
}

void setup() {
  Serial.begin(115200); // Start serial communication at 115200 baud rate
  MySerial.begin(9600, SERIAL_8N1, 17, 16);  // UART1 on GPIO 17 (TX) and GPIO 16 (RX)
  hlSerial.begin(4800, SERIAL_8N1, 23, 22); // Initialize Serial2 with baud rate 4800, RX=23, TX=22 (adjust if needed)

  // Initialize relay and PWM pins
  initializeRelays();
  initializePwmPins();

  Serial.println("Send a JSON object to control relays and PWM.");
}

void loop() {
  // Check if data is available to read
  if (Serial.available() > 0) {
    String receivedChar = Serial.readStringUntil('\n'); // Read data until newline
    lastSentTime = millis();

    // Parse and process the received JSON data
    StaticJsonDocument<400> receivedData;
    DeserializationError err = deserializeJson(receivedData, receivedChar);
    if (!err) {
      Serial.println("Received JSON:");
      Serial.println(receivedChar); // Print the received JSON string

      // Process the "toggle" section
      if (receivedData.containsKey("toggle")) {
        JsonObject toggle = receivedData["toggle"];
        for (JsonPair p : toggle) {
          int relayNum = atoi(p.key().c_str()); // Convert the key to an integer
          int relayState = p.value().as<int>(); // Get the state for the relay
          if (relayNum >= 1 && relayNum <= numRelays) {
            digitalWrite(relayPins[relayNum - 1], relayState == 1 ? HIGH : LOW);
            Serial.print("Relay "); Serial.print(relayNum); Serial.print(" set to "); Serial.println(relayState);
          } else {
            Serial.print("Invalid relay number: "); Serial.println(relayNum);
          }
          delay(500); // Add delay to trigger relays one by one
        }
      } else {
        Serial.println("No 'toggle' section found in JSON");
      }

      // Process the "regulate" section
      if (receivedData.containsKey("regulate")) {
        JsonObject regulate = receivedData["regulate"];
        for (JsonPair p : regulate) {
          int pwmNum = atoi(p.key().c_str()); // Convert the key to an integer
          int pwmValue = p.value().as<int>(); // Get the PWM value
          if (pwmNum >= 1 && pwmNum <= 2) {
            int dutyCycle = map(pwmValue, 0, 100, 0, 255);
            analogWrite(pwmPins[pwmNum - 1], dutyCycle);
            Serial.print("PWM "); Serial.print(pwmNum); Serial.print(" set to "); Serial.println(pwmValue);
          } else {
            Serial.print("Invalid PWM number: "); Serial.println(pwmNum);
          }
          delay(500); // Add delay to trigger relays one by one
        }
      } else {
        Serial.println("No 'regulate' section found in JSON");
      }
    } else {
      Serial.println("JSON parsing error");
    }
  }

  // Perform other tasks (e.g., updating feedbacks, etc.)
}
