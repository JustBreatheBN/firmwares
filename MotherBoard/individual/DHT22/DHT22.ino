//-----------------------Library_Import--------------------------------------
#include <Arduino.h>
#include "DHT.h"

#define DHTPIN 41
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

DHT dht(DHTPIN, DHTTYPE);

float tempC; // temperature in Celsius
float tempF; // temperature in Fahrenheit
float h, t, dt;

void setup() {
  Serial.begin(115200);
  dht.begin();
  
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void loop() {
  data_loop();
}
void data_loop() {
  h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println("Reading error");
  }
  
  String Data = "{";
  Data += "\"dtemp\":" + String(t) + ",";
  Data += "\"drh\":" + String(h) + "";
  Data += "}";
  Serial.println(Data);
  
  delay(2000);

}
