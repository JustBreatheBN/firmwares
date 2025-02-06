void setup() {
  // Initialize serial communication at 115200 baud rate
  Serial.begin(115200);

  // Configure GPIO 11 and GPIO 42 as input
  pinMode(11, INPUT_PULLUP);
  pinMode(42, INPUT                                                     );

  // Print initial instructions
  Serial.println("Checking the status of GPIO 11 and GPIO 42 every second.");
}

void loop() {
  // Check the status of GPIO 11
  int status11 = digitalRead(11);
  Serial.print("GPIO 11 is ");
  if (status11 == HIGH) {
    Serial.println("HIGH");
  } else {
    Serial.println("LOW");
  }

  // Check the status of GPIO 42
  int status42 = digitalRead(42);
  Serial.print("GPIO 42 is ");
  if (status42 == HIGH) {
    Serial.println("HIGH");
  } else {
    Serial.println("LOW");
  }

  // Wait for 1 second
  delay(1000);
}
