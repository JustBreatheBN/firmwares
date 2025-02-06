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
const int pwmPins[] = {32, 33}; // Declaring the PWM pins

// Number of relays and feedbacks
const int numRelays = sizeof(relayPins) / sizeof(relayPins[0]);
const int numFeedbacks = sizeof(feedbackPins) / sizeof(feedbackPins[0]);

String inputString = "";   // A string to hold incoming data
bool stringComplete = false;  // Whether the string is complete

// Function to initialize relay and feedback pins
void initializeRelays() {
  for (int i = 0; i < numRelays; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW); // Ensure all relays are off initially
  }
}

void initializeFeedbacks() {
  for (int i = 0; i < numFeedbacks; i++) {
    pinMode(feedbackPins[i], INPUT_PULLUP);
  }
}

// Function to initialize PWM pins
void initializePwmPins() {
  for (int i = 0; i < 2; i++) {
    pinMode(pwmPins[i], OUTPUT);
    analogWrite(pwmPins[i], 0); // Initialize PWM pins to 0
  }
}

// Function to control the relay
void controlRelay(int relayNumber, int state) {
  if (relayNumber >= 1 && relayNumber <= numRelays) {
    int gpio = relayPins[relayNumber - 1];
    digitalWrite(gpio, state);
  } else {
    Serial.print("Invalid relay number: ");
    Serial.println(relayNumber);
  }
}

// Function to control PWM
void controlPwm(int pwmNumber, int value) {
  if (pwmNumber >= 1 && pwmNumber <= 2) {
    int gpio = pwmPins[pwmNumber - 1];
    int dutyCycle = map(value, 1, 100, 0, 255); // Map 1-100 to 0-255
    analogWrite(gpio, dutyCycle);
  } else {
    Serial.print("Invalid PWM number: ");
    Serial.println(pwmNumber);
  }
}

// Function to manually parse and process the JSON input
void processJsonManually(String json) {
  // Extract and process "toggle" values
  int toggleIndex = json.indexOf("\"toggle\"");
  if (toggleIndex != -1) {
    int toggleStart = json.indexOf('{', toggleIndex);
    int toggleEnd = json.indexOf('}', toggleStart);
    String toggleString = json.substring(toggleStart + 1, toggleEnd);

    // Split the toggle string into individual relay controls
    int pos = 0;
    while ((pos = toggleString.indexOf(':')) != -1) {
      String relayNumber = toggleString.substring(0, pos).trim();
      int commaIndex = toggleString.indexOf(',');
      String state;
      if (commaIndex != -1) {
        state = toggleString.substring(pos + 1, commaIndex).trim();
        toggleString = toggleString.substring(commaIndex + 1);
      } else {
        state = toggleString.substring(pos + 1).trim();
        toggleString = "";
      }
      controlRelay(relayNumber.toInt(), state.toInt());
    }
  }

  // Extract and process "regulate" values
  int regulateIndex = json.indexOf("\"regulate\"");
  if (regulateIndex != -1) {
    int regulateStart = json.indexOf('{', regulateIndex);
    int regulateEnd = json.indexOf('}', regulateStart);
    String regulateString = json.substring(regulateStart + 1, regulateEnd);

    // Split the regulate string into individual PWM controls
    int pos = 0;
    while ((pos = regulateString.indexOf(':')) != -1) {
      String pwmNumber = regulateString.substring(0, pos).trim();
      int commaIndex = regulateString.indexOf(',');
      String value;
      if (commaIndex != -1) {
        value = regulateString.substring(pos + 1, commaIndex).trim();
        regulateString = regulateString.substring(commaIndex + 1);
      } else {
        value = regulateString.substring(pos + 1).trim();
        regulateString = "";
      }
      controlPwm(pwmNumber.toInt(), value.toInt());
    }
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

  // Set initial PWM values to 0
  controlPwm(1, 0);
  controlPwm(2, 0);
}

void loop() {
  // Check if a new serial input has arrived
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }

  // Process the input string when it is complete
  if (stringComplete) {
    processJsonManually(inputString);

    // Clear the string and reset the flag
    inputString = "";
    stringComplete = false;
  }

  // HLW8032 Energy Monitoring
  end_time = millis();
  HL.SerialReadLoop();

  if (end_time - start_time > SYNC_TIME) {
    if (HL.SerialRead == 1) {
      float voltage = HL.GetVol() * 0.001;
      float current = HL.GetCurrent();
      Serial.print("Voltage: ");
      Serial.print(voltage);
      Serial.print(" V, Current: ");
      Serial.print(current);
      Serial.println(" A");
    }
    start_time = end_time;
  }
}
