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

// Function to print all feedback states
void printFeedbacks() {
  Serial.println("Feedback states:");
  for (int i = 0; i < numFeedbacks; i++) {
    int feedbackState = digitalRead(feedbackPins[i]);
    Serial.print("Feedback from Relay ");
    Serial.print(i + 1);
    Serial.print(" on GPIO ");
    Serial.print(feedbackPins[i]);
    Serial.print(" is ");
    Serial.println(feedbackState == HIGH ? "HIGH" : "LOW");
  }
}


/////////////////PWM FUnctions

// Define the PWM pins
const int pwmPins[] = {32, 33};

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
void setup()
{
  Serial.begin(115200); // Start serial communication at 115200 baud rate
  MySerial.begin(9600, SERIAL_8N1, 17, 16);  // UART1 on GPIO 17 (TX) and GPIO 16 (RX)
  hlSerial.begin(4800, SERIAL_8N1, 23, 22); // Initialize Serial2 with baud rate 4800, RX=23, TX=22 (adjust if needed)
  // Initialize relay and feedback pins
  initializeRelays();
  initializeFeedbacks();

  Serial.println("Enter command in format: <Relay Number> <STATE> to control relay");
  Serial.println("Enter 'fb' to print feedback states");
  HL.begin(hlSerial, -1); // Initialize HLW8032 with Serial2 and IO pin 4 (change if needed)
  start_time = millis();

  // Initialize PWM pins                                                         
  initializePwmPins();

  Serial.println("Enter command in format: <PWM Number> <Value>");
  Serial.println("Example: 1 50 (to set PWM 1 to 50%)");
}

void loop()
{
  // Check if data is available to read
  if (Serial.available() > 0 ) {
    // Read the input
    String input = Serial.readStringUntil('\n');
    input.trim();

    // Check if the input is for relay control or feedback print
    if (input.equals("fb")) {
      // Print all feedback states
      printFeedbacks();
    }
    if (input.startsWith("p")) {
      // Find the positions of the spaces
      int firstSpaceIndex = input.indexOf(' ');
      int secondSpaceIndex = input.indexOf(' ', firstSpaceIndex + 1);

      // Extract the substrings and convert them to integers
      int pwmNumber = input.substring(firstSpaceIndex + 1, secondSpaceIndex).toInt();
      int value = input.substring(secondSpaceIndex + 1).toInt();

      Serial.print("PWM Number: ");
      Serial.println(pwmNumber);
      Serial.print("Value: ");
      Serial.println(value);
      // Control the PWM based on the input
      controlPwm(pwmNumber, value);
    }
    else if (input.startsWith("r")) {
      // Find the positions of the spaces
      int firstSpaceIndex = input.indexOf(' ');
      int secondSpaceIndex = input.indexOf(' ', firstSpaceIndex + 1);

      // Parse the input for relay control
      int relayNumber = input.substring(firstSpaceIndex + 1, secondSpaceIndex).toInt();
      int state = input.substring(secondSpaceIndex + 1).toInt();

      Serial.print("RELAY Number: ");
      Serial.println(relayNumber);
      Serial.print("STATE: ");
      Serial.println(state);
      // Control the relay based on the input
      controlRelay(relayNumber, state);
    }
  }
  if (MySerial.available() > 0) {
    String input = MySerial.readStringUntil('\n');
    input.trim();
    Serial.print("Recieved from Mother Board: ");
    Serial.println(input);
    // Check if the input is for relay control or feedback print
    if (input.equals("fb")) {
      // Print all feedback states
      printFeedbacks();
    }
    if (input.startsWith("p")) {
      // Find the positions of the spaces
      int firstSpaceIndex = input.indexOf(' ');
      int secondSpaceIndex = input.indexOf(' ', firstSpaceIndex + 1);

      // Extract the substrings and convert them to integers
      int pwmNumber = input.substring(firstSpaceIndex + 1, secondSpaceIndex).toInt();
      int value = input.substring(secondSpaceIndex + 1).toInt();

      Serial.print("PWM Number: ");
      Serial.println(pwmNumber);
      Serial.print("Value: ");
      Serial.println(value);
      // Control the PWM based on the input
      controlPwm(pwmNumber, value);
    }
    else if (input.startsWith("r")) {
      // Find the positions of the spaces
      int firstSpaceIndex = input.indexOf(' ');
      int secondSpaceIndex = input.indexOf(' ', firstSpaceIndex + 1);

      // Parse the input for relay control
      int relayNumber = input.substring(firstSpaceIndex + 1, secondSpaceIndex).toInt();
      int state = input.substring(secondSpaceIndex + 1).toInt();

      Serial.print("RELAY Number: ");
      Serial.println(relayNumber);
      Serial.print("STATE: ");
      Serial.println(state);
      // Control the relay based on the input
      controlRelay(relayNumber, state);
    }
    MySerial.println("Ack From Relay Board");
  }
  end_time = millis();
  HL.SerialReadLoop();

  if (end_time - start_time > SYNC_TIME)
  {
    if (HL.SerialRead == 1)
    {


      Serial.println("-----------------------------------");
      Serial.print("Voltage[V]: ");
      Serial.println(HL.GetVol() * 0.001, 2);

      Serial.print("Voltage Parameter: ");
      Serial.println(HL.GetVolAnalog(), 2);

      Serial.print("Current[A]: ");
      Serial.println(HL.GetCurrent(), 2);

      Serial.print("Current Parameter: ");
      Serial.println(HL.GetCurrentAnalog(), 2);

      Serial.print("Power[W]: ");
      Serial.println(HL.GetActivePower() * 0.001, 2);

      Serial.print("Inspecting Power: ");
      Serial.println(HL.GetInspectingPower(), 2);

      Serial.print("Power Factor: ");
      Serial.println(HL.GetPowerFactor(), 2);

      Serial.print("PF Register: ");
      Serial.println(HL.GetPF(), 2);

      Serial.print("PF Total: ");
      Serial.println(HL.GetPFAll(), 2);

      Serial.print("Energy[KWh]: ");
      Serial.println(HL.GetKWh());
    }
    start_time = end_time;
  }
}
