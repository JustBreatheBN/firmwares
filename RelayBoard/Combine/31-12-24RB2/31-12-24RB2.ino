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
unsigned long previousMillis = 0; // Store the last time the message was printed
const unsigned long interval = 60000; // 1 minute in milliseconds
// Define the GPIO pins for the relays and their feedback
const int relayPins[] = {25, 26, 27, 14, 12, 13, 15, 4, 2};
const int feedbackPins[] = {36, 39, 34, 35, 18, 19, 5, 21}; // GPIO pins for relay feedback
const int pwmPins[] = {32, 33}; // Declaring the PWM pins
//const int pwmPin = 13;  // GPIO pin to control PWM
const int pwmFrequency = 20000;  // 20 kHz frequency
const int pwmResolution = 8; // 8-bit resolution (0-255)
const int pwmChannel = 0;  // PWM channel


// Number of relays
const int numRelays = sizeof(relayPins) / sizeof(relayPins[0]);
const int numFeedbacks = sizeof(feedbackPins) / sizeof(feedbackPins[0]);

// Global variables for storing power data
float voltage_V = 0.0;
float current_A = 0.0;

unsigned long lastSentTime = 0;
StaticJsonDocument<512> relayStatusDoc;   // JSON document for relay status
StaticJsonDocument<200> regulateStatusDoc; // JSON document for regulate status

// PWM Gradual Increase Control Variables
struct PWMControl {
  int currentPWM = 0;      // Current PWM value
  int targetPWM = 0;       // Target PWM value
  bool isRamping = false;  // Is ramping active
  unsigned long lastUpdate = 0; // Last time PWM was updated
};

PWMControl pwmControl[2]; // For 2 PWM channels
const unsigned long pwmStepDelay = 4000; // 5 seconds per step


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
    if (ledcAttach(pwmPins[i], pwmFrequency, pwmResolution) == 0) {
      Serial.println("Error attaching PWM pin");
    } else {
      Serial.println("PWM pin attached successfully");
    }
//    int dutyCycle = map(0, 0, 100, 0, 255);  // Map 0-100 to 0-255
    ledcWrite(pwmPins[i], 0);  // Set PWM duty cycle
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

  Serial.println("{\"reboot\":1}");
  MySerial.println("{\"reboot\":1}");
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
    //    Serial.println("Sending feedback JSON:");
    serializeJson(outputDoc, Serial);
    serializeJson(outputDoc, MySerial);
    //    Serial.println(); // Add a newline for better readability
  } else if (receivedChar.equals("rb")) {
    // Print the current relay and PWM status
    //    Serial.println("Sending relay and PWM status JSON:");
    StaticJsonDocument<512> statusDoc;
    statusDoc["toggle"] = relayStatusDoc.as<JsonObject>(); // Include the toggle (relay) status
    statusDoc["regulate"] = regulateStatusDoc.as<JsonObject>(); // Include the PWM status
    serializeJson(statusDoc, Serial);
    serializeJson(statusDoc, MySerial);
    //    Serial.println(); // Add a newline for better readability
  } else if (receivedChar.equals("power")) {
    // Print the stored power data
    StaticJsonDocument<200> powerDoc;
    JsonObject power = powerDoc.createNestedObject("power");

    // Use the globally stored power data
    power["voltage_V"] = voltage_V;
    power["current_A"] = current_A;

    // Serialize and send the JSON response
    //    Serial.println("Sending power JSON:");
    serializeJson(powerDoc, Serial);
    serializeJson(powerDoc, MySerial);
    //    Serial.println(); // Add a newline for better readability
  } else if (receivedChar.equals("rbs")) {
    Serial.println("{\"isWorking\":1}");
    MySerial.println("{\"isWorking\":1}");
  }
  else {
    // Parse and process the received JSON data
    StaticJsonDocument<400> receivedData;
    DeserializationError err = deserializeJson(receivedData, receivedChar);
    if (!err) {
      //      Serial.println("Received JSON:");
      //      Serial.println(receivedChar); // Print the received JSON string

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
            //            Serial.print("Invalid relay number: "); Serial.println(relayNum);
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
            // Send status back immediately
            regulateStatusDoc[String(pwmNum)] = pwmValue;
            regulateOutput[String(pwmNum)] = pwmValue;
            Serial.print("Received PWM "); Serial.print(pwmNum); Serial.print(" target: "); Serial.println(pwmValue);

            // Start gradual PWM increase or decrease
            pwmControl[pwmNum - 1].targetPWM = pwmValue;
            pwmControl[pwmNum - 1].isRamping = true;
            pwmControl[pwmNum - 1].lastUpdate = millis();
            Serial.println("Gradual PWM adjustment initiated.");
          } else {
            Serial.print("Invalid PWM number: "); Serial.println(pwmNum);
          }
        }
      }

      // Serialize and print the final output JSON
      //      Serial.println("Final JSON after processing:");
      serializeJson(outputDoc, Serial);
      serializeJson(outputDoc, MySerial);
      //      Serial.println(); // Add a newline for better readability
    } else {
      //      Serial.println("JSON parsing error");
    }
  }
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Process input from Serial
  if (Serial.available() > 0) {
    String receivedChar = Serial.readStringUntil('\n');
    processInput(receivedChar);
  }

  // Process input from MySerial
  if (MySerial.available() > 0) {
    String receivedChar = MySerial.readStringUntil('\n');
    processInput(receivedChar);
  }

  // Handle gradual PWM updates
  handleGradualPWM();

  // Power data and periodic updates
  end_time = millis();
  HL.SerialReadLoop();
  if (end_time - start_time > SYNC_TIME) {
    if (HL.SerialRead == 1) {
      voltage_V = HL.GetVol() * 0.001;
      current_A = HL.GetCurrent();
    }
    start_time = end_time;
  }

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; 
    StaticJsonDocument<200> powerDoc;
    JsonObject power = powerDoc.createNestedObject("power");
    power["voltage_V"] = roundf(voltage_V * 100) / 100.0;
    power["current_A"] = roundf(current_A * 100) / 100.0;
    serializeJson(powerDoc, Serial);
    serializeJson(powerDoc, MySerial);
  }
}

void handleGradualPWM() {
  unsigned long currentMillis = millis();

  for (int i = 0; i < 2; i++) {
    if (pwmControl[i].isRamping) {
      if (currentMillis - pwmControl[i].lastUpdate >= pwmStepDelay) {
        pwmControl[i].lastUpdate = currentMillis; // Update the last time

        // Handle increasing PWM
        if (pwmControl[i].currentPWM < pwmControl[i].targetPWM) {
          pwmControl[i].currentPWM += 10;
          if (pwmControl[i].currentPWM > pwmControl[i].targetPWM) {
            pwmControl[i].currentPWM = pwmControl[i].targetPWM;
          }
        }

        // Handle decreasing PWM
        else if (pwmControl[i].currentPWM > pwmControl[i].targetPWM) {
          pwmControl[i].currentPWM -= 10;
          if (pwmControl[i].currentPWM < pwmControl[i].targetPWM) {
            pwmControl[i].currentPWM = pwmControl[i].targetPWM;
          }
        }

        // Apply the current PWM value to the pin
        int dutyCycle = map(pwmControl[i].currentPWM, 0, 100, 0, 255);
        analogWrite(pwmPins[i], dutyCycle);
        regulateStatusDoc[String(i + 1)] = pwmControl[i].currentPWM; // Update status

        Serial.print("PWM "); Serial.print(i + 1);
        Serial.print(" stepped to: "); Serial.println(pwmControl[i].currentPWM);

        // Stop ramping when target is reached
        if (pwmControl[i].currentPWM == pwmControl[i].targetPWM) {
          pwmControl[i].isRamping = false;
          Serial.print("PWM "); Serial.print(i + 1);
          Serial.println(" target reached.");
        }
      }
    }
  }
}
