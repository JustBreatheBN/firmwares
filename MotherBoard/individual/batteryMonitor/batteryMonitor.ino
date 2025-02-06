const int batteryPin = 15; // GPIO pin connected to the voltage divider output
const int relayPin = 16;   // GPIO pin connected to the relay control

void setup() {
  Serial.begin(115200);
  pinMode(batteryPin, INPUT);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW); // Ensure relay is off initially
}

void loop() {
  // Activate the relay to measure battery voltage
  digitalWrite(relayPin, HIGH);
  delay(100); // Small delay to stabilize the reading

  // Read the battery voltage
  int sensorValue = analogRead(batteryPin);

  // Deactivate the relay to avoid draining the battery
  digitalWrite(relayPin, LOW);

  // Convert the analog reading to a voltage value (considering the voltage divider)
  float voltage = sensorValue * (3.3 / 4095.0) * ((10.0 + 2.0) / 2.0);
  Serial.print("Battery Voltage: ");
  Serial.print(voltage);
  Serial.println(" V");

  delay(60000); // Wait 60 seconds before the next reading
}
