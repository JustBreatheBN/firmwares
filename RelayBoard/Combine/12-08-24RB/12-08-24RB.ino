#include <ArduinoJson.h>
#include "HLW8032.h"
#include <HardwareSerial.h>
#include <Arduino.h>

#define SYNC_TIME 60000
HardwareSerial hlSerial(1);  // Use UART1
#define hl_rs 23
#define hl_tx 22
static unsigned long start_time, end_time;
HLW8032 HL;
HardwareSerial MySerial(2);

// Define the GPIO pins for the relays and their feedback
const int relayPins[] = {25, 26, 27, 14, 12, 13, 15, 4, 2};
const int feedbackPins[] = {36, 39, 34, 35, 18, 19, 5, 21};

// Number of relays and feedbacks
const int numRelays = sizeof(relayPins) / sizeof(relayPins[0]);
const int numFeedbacks = sizeof(feedbackPins) / sizeof(feedbackPins[0]);

// Define the PWM pins
const int pwmPins[] = {32, 33};

// Create a JSON document
StaticJsonDocument<1024> jsonDoc;

void updateRelayJson() {
  JsonObject relays = jsonDoc["relays"];
  for (int i = 0; i < numRelays; i++) {
    relays[String(i + 1)] = digitalRead(relayPins[i]);
  }
}

void updateFeedbackJson() {
  JsonObject feedbacks = jsonDoc["feedbacks"];
  for (int i = 0; i < numFeedbacks; i++) {
    feedbacks[String(i + 1)] = digitalRead(feedbackPins[i]) == HIGH ? "HIGH" : "LOW";
  }
}

void updatePowerJson() {
  JsonObject power = jsonDoc["power"];
  power["voltage_V"] = HL.GetVol() * 0.001;
  power["current_A"] = HL.GetCurrent();
}

void printJson() {
  serializeJsonPretty(jsonDoc, Serial);
  Serial.println();
}

// Function to initialize relay and feedback pins
void initializeRelays() {
  for (int i = 0; i < numRelays; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW); // Ensure all relays are off initially
  }
  updateRelayJson();
}

void initializeFeedbacks() {
  for (int i = 0; i < numFeedbacks; i++) {
    pinMode(feedbackPins[i], INPUT_PULLUP);
  }
  updateFeedbackJson();
}

// Function to control the relay
void controlRelay(int relayNumber, int state) {
  if (relayNumber >= 1 && relayNumber <= numRelays) {
    int gpio = relayPins[relayNumber - 1];
    digitalWrite(gpio, state);
    updateRelayJson(); // Update JSON after changing the relay state
    printJson();
  } else {
    Serial.print("Invalid relay number: ");
    Serial.println(relayNumber);
  }
}

// Function to print all feedback states
void printFeedbacks() {
  updateFeedbackJson(); // Update JSON with feedback states
  printJson();
}

/////////////////PWM Functions

// Function to initialize PWM pins
void initializePwmPins() {
  for (int i = 0; i < 2; i++) {
    pinMode(pwmPins[i], OUTPUT);
  }
}

// Function to control PWM
void controlPwm(int pwmNumber, int value) {
  if (pwmNumber >= 1 && pwmNumber <= 2) {
    int gpio = pwmPins[pwmNumber - 1];
    int dutyCycle = map(value, 1, 100, 0, 255); // Map 1-100 to 0-255
    analogWrite(gpio, dutyCycle);
    Serial.print("PWM ");
    Serial.print(pwmNumber);
    Serial.print(" on GPIO ");
    Serial.print(gpio);
    Serial.print(" set to ");
    Serial.println(value);
  } else {
    Serial.print("Invalid PWM number: ");
    Serial.println(pwmNumber);
  }
}

/////////////////////////////PWM END

void setup() {
  Serial.begin(115200); // Start serial communication at 115200 baud rate
  MySerial.begin(9600, SERIAL_8N1, 17, 16);  // UART1 on GPIO 17 (TX) and GPIO 16 (RX)
  hlSerial.begin(4800, SERIAL_8N1, 23, 22); // Initialize Serial2 with baud rate 4800, RX=23, TX=22 (adjust if needed)

  // Initialize relay and feedback pins
  initializeRelays();
  initializeFeedbacks();

  // Initialize PWM pins
  initializePwmPins();

  // Initialize JSON structure
  jsonDoc["relays"] = JsonObject();
  jsonDoc["feedbacks"] = JsonObject();
  jsonDoc["power"] = JsonObject();

  Serial.println("Enter command in format: <Relay Number> <STATE> to control relay");
  Serial.println("Enter 'fb' to print feedback states");
  Serial.println("Enter command in format: <PWM Number> <Value>");
  Serial.println("Example: 1 50 (to set PWM 1 to 50%)");

  HL.begin(hlSerial, -1); // Initialize HLW8032 with Serial2 and IO pin 4 (change if needed)
  start_time = millis();
}

void loop() {
  // Check if data is available to read
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equals("fb")) {
      printFeedbacks(); // Print all feedback states
    }
    else if (input.startsWith("p")) {
      int firstSpaceIndex = input.indexOf(' ');
      int secondSpaceIndex = input.indexOf(' ', firstSpaceIndex + 1);
      int pwmNumber = input.substring(firstSpaceIndex + 1, secondSpaceIndex).toInt();
      int value = input.substring(secondSpaceIndex + 1).toInt();
      controlPwm(pwmNumber, value);
    }
    else if (input.startsWith("r")) {
      int firstSpaceIndex = input.indexOf(' ');
      int secondSpaceIndex = input.indexOf(' ', firstSpaceIndex + 1);
      int relayNumber = input.substring(firstSpaceIndex + 1, secondSpaceIndex).toInt();
      int state = input.substring(secondSpaceIndex + 1).toInt();
      controlRelay(relayNumber, state);
    }
  }

  if (MySerial.available() > 0) {
    String input = MySerial.readStringUntil('\n');
    input.trim();
    Serial.print("Received from Mother Board: ");
    Serial.println(input);

    if (input.equals("fb")) {
      printFeedbacks(); // Print all feedback states
    }
    else if (input.startsWith("p")) {
      int firstSpaceIndex = input.indexOf(' ');
      int secondSpaceIndex = input.indexOf(' ', firstSpaceIndex + 1);
      int pwmNumber = input.substring(firstSpaceIndex + 1, secondSpaceIndex).toInt();
      int value = input.substring(secondSpaceIndex + 1).toInt();
      controlPwm(pwmNumber, value);
    }
    else if (input.startsWith("r")) {
      int firstSpaceIndex = input.indexOf(' ');
      int secondSpaceIndex = input.indexOf(' ', firstSpaceIndex + 1);
      int relayNumber = input.substring(firstSpaceIndex + 1, secondSpaceIndex).toInt();
      int state = input.substring(secondSpaceIndex + 1).toInt();
      controlRelay(relayNumber, state);
    }
    MySerial.println("Ack From Relay Board");
  }

  end_time = millis();
  HL.SerialReadLoop();

  if (end_time - start_time > SYNC_TIME) {
    if (HL.SerialRead == 1) {
      updatePowerJson(); // Update JSON with power measurements
      printJson();
    }
    start_time = end_time;
  }
}
