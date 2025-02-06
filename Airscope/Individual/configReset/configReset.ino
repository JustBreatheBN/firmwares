const int buttonPin = 4; // GPIO pin connected to the push button
const unsigned long longPressTime = 3000; // 3 seconds in milliseconds

unsigned long buttonPressTime = 0;
bool buttonState = HIGH;
bool lastButtonState = HIGH;

void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP); // Set the button pin as input with internal pull-up resistor
}

void loop() {
  buttonState = digitalRead(buttonPin);

  if (buttonState == LOW && lastButtonState == HIGH) { // Button pressed
    buttonPressTime = millis();
  } else if (buttonState == HIGH && lastButtonState == LOW) { // Button released
    if (millis() - buttonPressTime >= longPressTime) {
      Serial.println("Button long pressed for 3 seconds");
    }
  }

  lastButtonState = buttonState; // Update the last button state
}
