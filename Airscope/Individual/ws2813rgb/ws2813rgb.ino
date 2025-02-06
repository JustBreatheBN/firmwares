#include <Adafruit_NeoPixel.h>

#define PIN            12
#define NUMPIXELS      1

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  pixels.begin(); // Initialize the NeoPixel library.
}

void loop() {
  // Red
  pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.show();
  delay(1000);

  // Green
  pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  pixels.show();
  delay(1000);

  // Blue
  pixels.setPixelColor(0, pixels.Color(0, 0, 255));
  pixels.show();
  delay(1000);
}
