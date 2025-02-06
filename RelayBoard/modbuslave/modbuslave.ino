#include <Arduino.h>
#include <HardwareSerial.h>
#include <ModbusRTUSlave.h>

// Relay and feedback pin definitions
const byte relayPins[8] = {25, 26, 27, 14, 12, 13, 15, 4};
const byte feedbackPins[8] = {36, 39, 34, 35, 18, 19, 5, 21};
const byte dePin = NO_DE_PIN; // No DE pin needed

HardwareSerial MySerial(1); // UART1
const int MySerialRX = 16;
const int MySerialTX = 17;

ModbusRTUSlave modbus(MySerial, dePin);  // Custom software serial port, no driver enable pin for RS-485

// Modbus data arrays
bool coils[8];
bool discreteInputs[8];
uint16_t holdingRegisters[3];  // For power monitoring: voltage, current, power
uint16_t inputRegisters[8];    // For relay feedback

void setup() {
  Serial.begin(115200);
  MySerial.begin(9600, SERIAL_8N1, MySerialRX, MySerialTX);

  Serial.println("Initializing...");

  // Initialize relay and feedback pins
  for (int i = 0; i < 8; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW);
    pinMode(feedbackPins[i], INPUT_PULLDOWN);
  }

  // Initialize Modbus
  modbus.configureCoils(coils, 8);
  modbus.configureDiscreteInputs(discreteInputs, 8);
  modbus.configureHoldingRegisters(holdingRegisters, 3);
  modbus.configureInputRegisters(inputRegisters, 8);
  modbus.begin(1, 38400);  // Slave address set to 1

  Serial.println("Initialization complete.");
}

void loop() {
  // Update power monitoring registers
  holdingRegisters[0] = readVoltage(); // Function to read voltage
  holdingRegisters[1] = readCurrent(); // Function to read current
  holdingRegisters[2] = readPower();   // Function to read power

  // Update relay feedback
  for (int i = 0; i < 8; i++) {
    inputRegisters[i] = digitalRead(feedbackPins[i]);
  }

  // Poll Modbus
  modbus.poll()
  Serial.println("Modbus connected and data received.");


  // Update relay states based on Modbus coils
  for (int i = 0; i < 8; i++) {
    digitalWrite(relayPins[i], coils[i] ? HIGH : LOW);
    Serial.print("Relay ");
    Serial.print(i + 1);
    Serial.print(" state: ");
    Serial.println(coils[i] ? "ON" : "OFF");
  }

  // Print power monitoring values
  Serial.print("Voltage: ");
  Serial.println(holdingRegisters[0]);
  Serial.print("Current: ");
  Serial.println(holdingRegisters[1]);
  Serial.print("Power: ");
  Serial.println(holdingRegisters[2]);

  // Print relay feedback
  for (int i = 0; i < 8; i++) {
    Serial.print("Relay ");
    Serial.print(i + 1);
    Serial.print(" feedback: ");
    Serial.println(inputRegisters[i]);
  }

  delay(1000); // Adjust the delay as needed
}

// Dummy functions for power monitoring
uint16_t readVoltage() {
  // Replace with actual voltage reading code
  return 220; // Example value
}

uint16_t readCurrent() {
  // Replace with actual current reading code
  return 10; // Example value
}

uint16_t readPower() {
  // Replace with actual power reading code
  return 2200; // Example value
}
