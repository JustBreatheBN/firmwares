//-----------------------Library_Import--------------------------------------
#include <Arduino.h>
#include <ArduinoJson.h>
#include "DFRobot_OzoneSensor.h"
#include "DFRobot_OxygenSensor.h"

#define Ozone_IICAddress OZONE_ADDRESS_0

DFRobot_OzoneSensor Ozone;

#define COLLECT_NUMBER  10

void setup() {

  Serial.begin(115200);
  Wire.begin();
  while(!Ozone.begin(Ozone_IICAddress)){
    Serial.println("I2c device number error !");
    delay(1000);
  }
  Serial.println("Ozone Found success !");
  /**
   * set measuer mode
   * MEASURE_MODE_AUTOMATIC         active  mode
   * MEASURE_MODE_PASSIVE           passive mode
   */
  Ozone.setModes(MEASURE_MODE_PASSIVE);
  delay(2000);
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void loop() {
  StaticJsonDocument<300> doc;
  doc["O3"] = Ozone.readOzoneData(COLLECT_NUMBER);
  serializeJson(doc, Serial);
  Serial.println();
  delay(5000);
}
