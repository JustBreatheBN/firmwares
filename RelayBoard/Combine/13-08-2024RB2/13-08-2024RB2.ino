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
const int feedbackPins[] = {36, 39, 34, 35, 18, 19, 5, 21}; // GPIO pins for relay feedback
const int pwmPins[] = {32, 33}; // Declaring the PWM pins

// Number of relays
const int numRelays = sizeof(relayPins) / sizeof(relayPins[0]);
const int numFeedbacks = sizeof(feedbackPins) / sizeof(feedbackPins[0]);

// Global variables for storing power data
float voltage_V = 0.0;
float current_A = 0.0;

unsigned long lastSentTime = 0;
StaticJsonDocument<512> relayStatusDoc;   // JSON document for relay status
StaticJsonDocument<200> regulateStatusDoc; // JSON document for regulate status

void initializeRelays() {
  for (int i = 0; i < numRelays; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW); // Ensure all relays are off initially
    relayStatusDoc[String(i + 1)] = 0; // Initialize relay status in JSON
  }
}

void initializeFeedbacks() {
  for (int i = 0; i < numFeedbacks; i++) {
    pinMode(feedbackPins[i], INPUT_PULLUP);
  }
}

void initializePwmPins() {
  for (int i = 0; i < 2; i++) {
    pinMode(pwmPins[i], OUTPUT);
    analogWrite(pwmPins[i], 0); // Initialize PWM pins to 0
    regulateStatusDoc[String(i + 1)] = 0; // Initialize PWM status in JSON
  }
}

void setup() {
  Serial.begin(115200); // Start serial communication at 115200 baud rate
  MySerial.begin(9600, SERIAL_8N1, 17, 16);  // UART1 on GPIO 17 (TX) and GPIO 16 (RX)
  hlSerial.begin(4800, SERIAL_8N1, 23, 22); // Initialize Serial2 with baud rate 4800, RX=23, TX=22 (adjust if needed)

  // Initialize relay, feedback, and PWM pins
  initializeRelays();
  initializeFeedbacks();
  initializePwmPins();

  Serial.println("Send a JSON object to control relays and PWM.");
  HL.begin(hlSerial, -1); // Initialize HLW8032 with Serial2 and IO pin 4 (change if needed)
  start_time = millis();
}

void processInput(String receivedChar) {
  // JSON document for the final output after processing
  StaticJsonDocument<512> outputDoc;

  // Check for various commands
  if (receivedChar.equals("fb")) {
    JsonObject feedback = outputDoc.createNestedObject("feedback");

    // Gather relay feedback
    for (int i = 0; i < numFeedbacks; i++) {
      feedback[String(i + 1)] = digitalRead(feedbackPins[i]) == LOW ? 1 : 0;
    }

    // Serialize and send the JSON response
    Serial.println("Sending feedback JSON:");
    serializeJson(outputDoc, Serial);
    Serial.println(); // Add a newline for better readability
  } else if (receivedChar.equals("rb")) {
    // Print the current relay and PWM status
    Serial.println("Sending relay and PWM status JSON:");
    StaticJsonDocument<512> statusDoc;
    statusDoc["toggle"] = relayStatusDoc.as<JsonObject>(); // Include the toggle (relay) status
    statusDoc["regulate"] = regulateStatusDoc.as<JsonObject>(); // Include the PWM status
    serializeJson(statusDoc, Serial);
    Serial.println(); // Add a newline for better readability
  } else if (receivedChar.equals("power")) {
    // Print the stored power data
    StaticJsonDocument<200> powerDoc;
    JsonObject power = powerDoc.createNestedObject("power");

    // Use the globally stored power data
    power["voltage_V"] = voltage_V;
    power["current_A"] = current_A;

    // Serialize and send the JSON response
    Serial.println("Sending power JSON:");
    serializeJson(powerDoc, Serial);
    Serial.println(); // Add a newline for better readability
  } else {
    // Parse and process the received JSON data
    StaticJsonDocument<400> receivedData;
    DeserializationError err = deserializeJson(receivedData, receivedChar);
    if (!err) {
      Serial.println("Received JSON:");
      Serial.println(receivedChar); // Print the received JSON string

      // Process the "toggle" section
      if (receivedData.containsKey("toggle")) {
        JsonObject toggleOutput = outputDoc.createNestedObject("toggle");
        JsonObject toggle = receivedData["toggle"];
        for (JsonPair p : toggle) {
          int relayNum = atoi(p.key().c_str()); // Convert the key to an integer
          int relayState = p.value().as<int>(); // Get the state for the relay
          if (relayNum >= 1 && relayNum <= numRelays) {
            digitalWrite(relayPins[relayNum - 1], relayState == 1 ? HIGH : LOW);
            delay(500); // Add delay to trigger relays one by one

            // Check feedback to confirm relay is working as expected
            if ((relayState == 1 && digitalRead(feedbackPins[relayNum - 1]) == LOW) ||
                (relayState == 0 && digitalRead(feedbackPins[relayNum - 1]) == HIGH)) {
              relayStatusDoc[String(relayNum)] = relayState; // Relay is working as expected
              toggleOutput[String(relayNum)] = relayState;
            } else {
              relayStatusDoc[String(relayNum)] = -1; // Relay did not trigger correctly
              toggleOutput[String(relayNum)] = -1;
            }
          } else {
            Serial.print("Invalid relay number: "); Serial.println(relayNum);
          }
        }
      }

      // Process the "regulate" section
      if (receivedData.containsKey("regulate")) {
        JsonObject regulateOutput = outputDoc.createNestedObject("regulate");
        JsonObject regulate = receivedData["regulate"];
        for (JsonPair p : regulate) {
          int pwmNum = atoi(p.key().c_str()); // Convert the key to an integer
          int pwmValue = p.value().as<int>(); // Get the PWM value
          if (pwmNum >= 1 && pwmNum <= 2) {
            int dutyCycle = map(pwmValue, 0, 100, 0, 255);
            analogWrite(pwmPins[pwmNum - 1], dutyCycle);
            regulateStatusDoc[String(pwmNum)] = pwmValue; // Store PWM status
            regulateOutput[String(pwmNum)] = pwmValue;
            Serial.print("PWM "); Serial.print(pwmNum); Serial.print(" set to "); Serial.println(pwmValue);
          } else {
            Serial.print("Invalid PWM number: "); Serial.println(pwmNum);
          }
        }
      }

      // Serialize and print the final output JSON
      Serial.println("Final JSON after processing:");
      serializeJson(outputDoc, Serial);
      Serial.println(); // Add a newline for better readability
    } else {
      Serial.println("JSON parsing error");
    }
  }
}

void loop() {
  // Process input from Serial
  if (Serial.available() > 0) {
    String receivedChar = Serial.readStringUntil('\n'); // Read data until newline
    processInput(receivedChar);
  }

  // Process input from MySerial
  if (MySerial.available() > 0) {
    String receivedChar = MySerial.readStringUntil('\n'); // Read data until newline
    processInput(receivedChar);
  }
  end_time = millis();
  // Update power data when available
  HL.SerialReadLoop();
  if (end_time - start_time > SYNC_TIME) {
    if (HL.SerialRead == 1) {
      // Store the power data globally
      voltage_V = HL.GetVol() * 0.001;
      current_A = HL.GetCurrent();
    }
    start_time = end_time;
  }
}
