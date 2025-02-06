//-----------------------Library_Import--------------------------------------
#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include "DFRobot_OxygenSensor.h"

#define Oxygen_IICAddress ADDRESS_3

DFRobot_OxygenSensor oxygen;

#define COLLECT_NUMBER  10

void setup() {

  Serial.begin(115200);
  Wire.begin();
  while(!oxygen.begin(Oxygen_IICAddress)){
    Serial.println("I2c device number error !");
    delay(1000);
  }
  Serial.println("Oxyen Found success !");
  delay(2000);
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void loop() {
  StaticJsonDocument<300> doc;
  doc["O2"] = oxygen.getOxygenData(COLLECT_NUMBER);
  serializeJson(doc, Serial);
  Serial.println();
  delay(5000);
}
