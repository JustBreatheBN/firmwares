#include <Arduino.h>

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

void setup() {
    // Initialize serial communication
    Serial.begin(115200);

    // Initialize PWM pins
    initializePwmPins();

    Serial.println("Enter command in format: <PWM Number> <Value>");
    Serial.println("Example: 1 50 (to set PWM 1 to 50%)");
}

void loop() {
    // Check if data is available to read
    if (Serial.available() > 0) {
        // Read the input
        String input = Serial.readStringUntil('\n');

        // Parse the input
        int pwmNumber = input.substring(0, input.indexOf(' ')).toInt();
        int value = input.substring(input.indexOf(' ') + 1).toInt();

        // Control the PWM based on the input
        controlPwm(pwmNumber, value);
    }
}
