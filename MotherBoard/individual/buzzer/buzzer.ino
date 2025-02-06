#include <Arduino.h>  // Include core Arduino functionalities and definitions

// Define the buzzer pin
const int buzzerPin = 46;

// Function to beep the buzzer 5 times
void beepBuzzer() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(buzzerPin, HIGH); // Turn buzzer on
    delay(500);                    // Wait for 500ms
    digitalWrite(buzzerPin, LOW);  // Turn buzzer off
    delay(500);                    // Wait for 500ms
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize the buzzer pin
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, HIGH); // Ensure buzzer is off initially
}

void loop() {
  //    // Call the beepBuzzer function to beep the buzzer 5 times
      beepBuzzer();
      Serial.println("Buzzer");
  //    // Add a delay to prevent continuous beeping
      delay(5000); // Wait 5 seconds before beeping again
//  digitalWrite(buzzerPin, HIGH);
}
