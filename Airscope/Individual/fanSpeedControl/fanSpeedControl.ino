#include <Arduino.h>
#include "driver/ledc.h"  // Explicitly include LEDC driver

const int pwmPin = 13;  // GPIO pin to control PWM
const int pwmFrequency = 20000;  // 20 kHz frequency
const int pwmResolution = 8; // 8-bit resolution (0-255)
const int pwmChannel = 0;  // PWM channel

void setup() {
  Serial.begin(115200);

  // Configure PWM channel and attach the pin
  if (ledcAttach(pwmPin, pwmFrequency, pwmResolution) == 0) {
    Serial.println("Error attaching PWM pin");
  } else {
    Serial.println("PWM pin attached successfully");
  }

  Serial.println("Enter PWM value (0-100):");
}

void loop() {
  if (Serial.available() > 0) {
    int pwmValue = Serial.parseInt();  // Read the input value

    // Check if the value is within the range
    if (pwmValue >= 0 && pwmValue <= 100) {
      int dutyCycle = map(pwmValue, 0, 100, 0, 255);  // Map 0-100 to 0-255
      ledcWrite(pwmPin, dutyCycle);  // Set PWM duty cycle
      Serial.print("PWM Value: ");
      Serial.println(pwmValue);
      Serial.print("Duty Cycle: ");
      Serial.println(dutyCycle);
    } else {
      Serial.println("Invalid input. Enter a value between 0 and 100.");
    }
  }
}
