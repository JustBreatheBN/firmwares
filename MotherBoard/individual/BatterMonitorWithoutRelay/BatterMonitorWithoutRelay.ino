// Define GPIO pin and other parameters
const int analogPin = 15; // GPIO 15 for ADC input
const float referenceVoltage = 3.3; // ESP32 ADC reference voltage is 3.3V
const int adcResolution = 4095; // 12-bit ADC resolution
const float voltageDividerRatio = 20.0; // Step down from 12V to 0.6V

void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);
  
  // Setup GPIO 15 as an ADC pin
  pinMode(analogPin, INPUT);
  
  // Wait for Serial to be ready
  while (!Serial) { delay(10); }
}

void loop() {
  // Read the raw ADC value (0 to 4095)
  int adcValue = analogRead(analogPin);
  
  // Calculate the voltage at the GPIO pin (0.6V in the case of a 12V input)
  float measuredVoltage = (adcValue / (float)adcResolution) * referenceVoltage;
  
  // Calculate the original voltage (12V) based on the voltage divider ratio
  float actualVoltage = measuredVoltage * voltageDividerRatio;
  
  // Print the ADC value and the calculated voltage
  Serial.print("ADC Value: ");
  Serial.print(adcValue);
  Serial.print(" | Measured Voltage: ");
  Serial.print(measuredVoltage, 3); // Voltage at GPIO
  Serial.print("V | Actual Voltage: ");
  Serial.print(actualVoltage, 3); // Calculated original voltage
  Serial.println("V");

  // Delay for a second before the next reading
  delay(1000);
}
