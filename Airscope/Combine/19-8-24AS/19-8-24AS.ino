#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "DHT.h"
#include <BH1750.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Adafruit_SGP30.h"
#include "DFRobot_MICS.h"
#include "DFRobot_OxygenSensor.h"
#include "DFRobot_OzoneSensor.h"
#define Oxygen_IICAddress ADDRESS_3
#define Ozone_IICAddress OZONE_ADDRESS_0
#include "PMS.h"
#include <SoftwareSerial.h>
DFRobot_OxygenSensor oxygen;
DFRobot_OzoneSensor Ozone;
Adafruit_SGP30 sgp30;
#define COLLECT_NUMBER  10

#define MICS_I2C_ADDRESS MICS_ADDRESS_0

DFRobot_MICS_I2C mics(&Wire, MICS_I2C_ADDRESS);


BH1750 lightMeter;

#define DHTPIN 27
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

DHT dht(DHTPIN, DHTTYPE);

float tempC; // temperature in Celsius
float tempF; // temperature in Fahrenheit
float h, t, dt;

const int oneWireBus = 14;     

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

#define PCBARTISTS_DBM 0x48

#define I2C_REG_VERSION      0x00
#define I2C_REG_ID3          0x01
#define I2C_REG_ID2          0x02
#define I2C_REG_ID1          0x03
#define I2C_REG_ID0          0x04
#define I2C_REG_SCRATCH      0x05
#define I2C_REG_CONTROL      0x06
#define I2C_REG_TAVG_HIGH    0x07
#define I2C_REG_TAVG_LOW     0x08
#define I2C_REG_RESET        0x09
#define I2C_REG_DECIBEL      0x0A
#define I2C_REG_MIN          0x0B
#define I2C_REG_MAX          0x0C
#define I2C_REG_THR_MIN      0x0D
#define I2C_REG_THR_MAX      0x0E
#define I2C_REG_HISTORY_0    0x14
#define I2C_REG_HISTORY_99   0x77

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

#define sw_rs 32
#define sw_tx 33
EspSoftwareSerial::UART SoftSerial;

#define pms_serial SoftSerial
PMS pms(pms_serial);

//------------------------PM25AQI_initialize-----------------------------------------------------
void PM25AQI_init() {
  pms_serial.begin(9600, EspSoftwareSerial::SWSERIAL_8N1, sw_rs, sw_tx, false, 256);
  // high speed half duplex, turn off interrupts during tx
  pms_serial.enableIntTx(false);
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
PMS::DATA pms_data;
int pms25_data[3];
void update_pms_data() {
  pms_serial.listen();
  pms.readUntil(pms_data);
  pms25_data[0] = pms_data.PM_AE_UG_1_0;
  pms25_data[1] = pms_data.PM_AE_UG_2_5;
  pms25_data[2] = pms_data.PM_AE_UG_10_0;
}
//-----------------------------------------------------------------------------
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
  Serial2.begin(9600, SERIAL_8N1,16,17); // Initialize Serial2 for HC-12 communication
  Wire.begin();
  
  lightMeter.begin();
  byte version = reg_read(PCBARTISTS_DBM, I2C_REG_VERSION);
  Serial.print("dbMeter VERSION = 0x");
  Serial.println(version, HEX);

  byte id[4];
  id[0] = reg_read(PCBARTISTS_DBM, I2C_REG_ID3);
  id[1] = reg_read(PCBARTISTS_DBM, I2C_REG_ID2);
  id[2] = reg_read(PCBARTISTS_DBM, I2C_REG_ID1);
  id[3] = reg_read(PCBARTISTS_DBM, I2C_REG_ID0);
  dht.begin();
  sensors.begin();
  mics.begin();
  uint8_t mode = mics.getPowerState();
  if (mode == SLEEP_MODE) {
    mics.wakeUpMode();
    Serial.println("wake up sensor success!");
  } else {
    Serial.println("The MICS sensor is wake up mode");
  }
  oxygen.begin(Oxygen_IICAddress);
  Ozone.begin(Ozone_IICAddress);
  Ozone.setModes(MEASURE_MODE_PASSIVE);
  PM25AQI_init();
  sgp30_init();
  Serial.println(F("started"));
}

void loop() {
  StaticJsonDocument<1024> doc;
  sensors.requestTemperatures(); 
  update_pms_data();
  float lux = luxLevel();
  int Co2 = fetchCo2Ppm();  // in ppm
  byte sound_level = reg_read(PCBARTISTS_DBM, I2C_REG_DECIBEL);
  float temperatureC = sensors.getTempCByIndex(0);
  h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    t = 25;
    h = 50;
  }
  float sgp30_TVOC = get_sgp30_TVOC();
  float sgp30_CO2 = get_sgp30_CO2();
//  doc["id"] = "DDKit";
  doc["lux"] = lux;
  doc["co2"] = Co2;
  doc["db"] = sound_level;
  doc["temperature1"] = t;
  doc["humidity"] = h;
  doc["temperature2"] = temperatureC;
  doc["co"] = mics.getGasData(CO);
  doc["no2"] = mics.getGasData(NO2);
  doc["nh3"] = mics.getGasData(NH3);
  doc["o2"] = oxygen.getOxygenData(COLLECT_NUMBER);
  doc["o3"] = Ozone.readOzoneData(COLLECT_NUMBER);
  doc["pm1"] = pms25_data[0];
  doc["pm2_5"] = pms25_data[1];
  doc["pm10"] = pms25_data[2];
  doc["tvoc"] = sgp30_TVOC;
  doc["eco2"] = sgp30_CO2;
  serializeJsonPretty(doc, Serial);
  serializeJsonPretty(doc, Serial2);
  delay(2000);  
}

float luxLevel(){
  return lightMeter.readLightLevel();
}

byte reg_read(byte addr, byte reg)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, (uint8_t)1);
  byte data = Wire.read();
  return data;
}
