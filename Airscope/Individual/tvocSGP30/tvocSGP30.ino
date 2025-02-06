//-----------------------Library_Import--------------------------------------
#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SGP30.h"
#include <ArduinoJson.h>

Adafruit_SGP30 sgp30;
//----------------------------------------------------------------------------
//------------------------SGP_30----------------------------------------------------
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
  // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature));  // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);                                                                 // [mg/m^3]
  return absoluteHumidityScaled;
}
//void set_sgp30_ht()
//{
// If you have a temperature / humidity sensor, you can set the absolute humidity to enable the humditiy compensation for the air quality signals
//float temperature = 22.1; // [°C]
//float humidity = 45.2; // [%RH]
//sgp30.setHumidity(getAbsoluteHumidity(temperature, humidity));
//}
void sgp30_init() {
  if (!sgp30.begin()) {
    Serial.println("Sensor not found :(");
    while (1)
      ;
  }
  Serial.print("Found SGP30 serial #");
  Serial.print(sgp30.serialnumber[0], HEX);
  Serial.print(sgp30.serialnumber[1], HEX);
  Serial.println(sgp30.serialnumber[2], HEX);

  // If you have a baseline measurement from before you can assign it to start, to 'self-calibrate'
  //sgp30.setIAQBaseline(0x8E68, 0x8F41);  // Will vary for each sensor!
}

float get_sgp30_TVOC() {
  if (!sgp30.IAQmeasure()) {
    Serial.println("Measurement failed");
    return (0);
  }
  return (sgp30.TVOC);  //Serial.print(" ppb\t");
}

float get_sgp30_CO2() {
  if (!sgp30.IAQmeasure()) {
    Serial.println("Measurement failed");
    return (0);
  }
  return (sgp30.eCO2);  //Serial.println(" ppm");
}
uint16_t TVOC_base, eCO2_base;
float get_sgp30_eCO2_base() {
  if (!sgp30.getIAQBaseline(&eCO2_base, &TVOC_base)) {
    Serial.println("Failed to get baseline readings");
    return (0);
  }
  return (eCO2_base);  //Serial.println(" ppm");
}
float get_sgp30_TVOC_base() {
  if (!sgp30.getIAQBaseline(&eCO2_base, &TVOC_base)) {
    Serial.println("Failed to get baseline readings");
    return (0);
  }
  return (TVOC_base);  //Serial.println(" ppm");
}


void setup() {
  
  Serial.begin(115200);
  Wire.begin();
  sgp30_init();
  Serial.println("ok sgp30");  //1
  delay(2000);
}

void loop() {
  StaticJsonDocument<300> doc;
  float sgp30_TVOC = get_sgp30_TVOC();
  float sgp30_CO2 = get_sgp30_CO2();
  float sgp30_eCO2_base = get_sgp30_eCO2_base();
  float sgp30_TVOC_base = get_sgp30_TVOC_base();

  doc["tvoc"] = sgp30_TVOC;
  doc["eco2"] = sgp30_CO2;
  doc["tvocbase"] = sgp30_TVOC_base;
  doc["eco2base"] = sgp30_eCO2_base;
  serializeJson(doc, Serial);
  Serial.println("------------------");
  delay(5000);
}
