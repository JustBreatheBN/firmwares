//-----------------------Library_Import--------------------------------------
#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>

//-----------------------------------------------------------------------------
//----------------------------DS-CO2-20----------------------------------------
unsigned int fetchCo2Ppm() {
  unsigned char CO2_Read[] = { 0x42, 0x4d, 0xe3, 0x00, 0x00, 0x01, 0x72 };
  unsigned char CO2_Value[24], num = 0;
//  Serial.print("CO2_Value=");
  Wire.requestFrom(8, 12);  // request 6 bytes from peripheral device #8
  while (Wire.available()) {
    CO2_Value[num] = Wire.read();
//    Serial.print(CO2_Value[num], HEX);
//    Serial.print("\t");
    num++;
  }
  int co2_final = int(CO2_Value[4] * 256.0 + CO2_Value[5]);
  if (co2_final < 400)
    co2_final = 400;
  return (co2_final);
}


void setup() {
  
  Serial.begin(115200);
  Wire.begin();
  delay(2000);
}

void loop() {
  StaticJsonDocument<300> doc;
  int Co2 = fetchCo2Ppm();  // in ppm
  doc["co2"] = Co2;
  serializeJson(doc, Serial);
  Serial.println();
  delay(2000);
}
