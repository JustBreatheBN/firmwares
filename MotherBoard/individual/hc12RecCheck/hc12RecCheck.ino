#include <Arduino.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 13, 12); // Initialize Serial3 for HC-12 communication
}

void loop() {

  // HC12 input check
  if (Serial2.available() > 0) {
    String message = Serial2.readStringUntil('}');
    message = message + "}";
    Serial.print("HC12 Received: ");
//    Serial.println(message);

    // Parse and process the received JSON data
    StaticJsonDocument<400> receivedData;
    DeserializationError err = deserializeJson(receivedData, message);
    if (!err) {
      serializeJson(receivedData, Serial);
      Serial.println();
    } else {
      Serial.println("JSON parsing error");
    }
  }
  
}
