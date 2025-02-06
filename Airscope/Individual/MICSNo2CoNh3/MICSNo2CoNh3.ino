//-----------------------Library_Import--------------------------------------
#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>

#include "DFRobot_MICS.h"

#define MICS_I2C_ADDRESS MICS_ADDRESS_0

DFRobot_MICS_I2C mics(&Wire, MICS_I2C_ADDRESS);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  while (!mics.begin()) {
    Serial.println("MICS NOT Deivces !");
    delay(1000);
  } Serial.println("MICS Device connected successfully !");
  uint8_t mode = mics.getPowerState();
  if (mode == SLEEP_MODE) {
    mics.wakeUpMode();
    Serial.println("wake up sensor success!");
  } else {
    Serial.println("The MICS sensor is wake up mode");
  }
  delay(2000);
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void loop() {
  StaticJsonDocument<300> doc;
  doc["co"] = mics.getGasData(CO);
  doc["No2"] = mics.getGasData(NO2);
  doc["NH3"] = mics.getGasData(NH3);

  serializeJson(doc, Serial);
  Serial.println();
  delay(2000);
}
