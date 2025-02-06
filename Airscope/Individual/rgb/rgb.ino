#include <Adafruit_NeoPixel.h>

// Define the GPIO pin and number of LEDs
#define LED_PIN 12
#define NUM_LEDS 1

// Create a NeoPixel object
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  strip.begin();          // Initialize the NeoPixel library
  strip.show();           // Turn off all LEDs by initializing them
}

void loop() {
  // Cycle through the colors with a delay
  setColor(strip.Color(255, 0, 0), 1000); // Red
  setColor(strip.Color(0, 255, 0), 1000); // Green
  setColor(strip.Color(0, 0, 255), 1000); // Blue
}

// Function to set the color of the strip and wait
void setColor(uint32_t color, int wait) {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color); // Set pixel color
  }
  strip.show();                  // Update the strip
  delay(wait);                   // Wait for the specified time
}
