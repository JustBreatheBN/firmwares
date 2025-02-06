#include <Arduino.h>

// Define the RGB LED pins
const int redPin = 38;
const int greenPin = 39;
const int bluePin = 40;

// Function to set the color of the RGB LED
void setColor(int redValue, int greenValue, int blueValue) {
    analogWrite(redPin, redValue);
    analogWrite(greenPin, greenValue);
    analogWrite(bluePin, blueValue);
}

// Function to turn the Red LED on/off
void redOn() {
    setColor(255, 0, 0);  // Red on, Green and Blue off
}

void redOff() {
    setColor(0, 0, 0);    // All off
}

// Function to turn the Green LED on/off
void greenOn() {
    setColor(0, 255, 0);  // Green on, Red and Blue off
}

void greenOff() {
    setColor(0, 0, 0);    // All off
}

// Function to turn the Blue LED on/off
void blueOn() {
    setColor(0, 0, 255);  // Blue on, Red and Green off
}

void blueOff() {
    setColor(0, 0, 0);    // All off
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);

    // Initialize the RGB LED pins as outputs
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);

    // Start with the LED off
    setColor(0, 0, 0);
}

void loop() {
    // Test the colors

    // Red on for 1 second, then off
    redOn();
    delay(1000);
    redOff();
    delay(500);

    // Green on for 1 second, then off
    greenOn();
    delay(1000);
    greenOff();
    delay(500);

    // Blue on for 1 second, then off
    blueOn();
    delay(1000);
    blueOff();
    delay(500);
}
