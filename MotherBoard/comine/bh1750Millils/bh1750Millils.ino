//General Libraries
#include <Wire.h>
#include <Arduino.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <RTClib.h>
#include <HardwareSerial.h>

//Sensor Linraries
#include <BH1750.h>
#include "DHT.h"
#include "PMS.h"

//GPIO's
const int buzzerPin = 46;
const int buttonPin = 14; // GPIO pin connected to the push button
const int doorLatchSensorPin = 45;
#define DHTPIN 41
const int moisturePins[4] = {4, 5, 6, 7};
int moistureValues[4] = {0, 0, 0, 0}; // Array to store sensor readings

//Object
BH1750 lightMeter;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800, 60000); // IST offset 19800 seconds (5.5 hours)
RTC_DS3231 rtc;
#define PCBARTISTS_DBM 0x48
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);
HardwareSerial hwSerial(1);  // Use UART1
PMS pms(hwSerial);
//------------------------PM25AQI_initialize-----------------------------------------------------
#define hw_rs 18
#define hw_tx 17
void PM25AQI_init() {
  hwSerial.begin(9600, SERIAL_8N1, hw_rs, hw_tx);
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
PMS::DATA pms_data;
int pms25_data[3];
void update_pms_data() {
  if (pms.readUntil(pms_data)) {
    pms25_data[0] = pms_data.PM_AE_UG_1_0;
    pms25_data[1] = pms_data.PM_AE_UG_2_5;
    pms25_data[2] = pms_data.PM_AE_UG_10_0;
  }
}
//-----------------------------------------------------------------------------

//Variables//
//millis
const unsigned long interval = 2000; // Interval in milliseconds (1 second)
unsigned long previousMillis = 0;    // Store the last time the message was printed

//config Reset Button
const unsigned long longPressTime = 3000; // 3 seconds in milliseconds
unsigned long buttonPressTime = 0;
bool buttonState = HIGH;
bool lastButtonState = HIGH;

// Replace with your network credentials
const char* ssid = "Just Breathe";
const char* password = "9845055994";

//db sound level Variables
#define PCBARTISTS_DBM 0x48
#define I2C_REG_VERSION      0x00
#define I2C_REG_ID3          0x01
#define I2C_REG_ID2          0x02
#define I2C_REG_ID1          0x03
#define I2C_REG_ID0          0x04
#define I2C_REG_DECIBEL      0x0A


//DHT22 temp and rh
float h, t;

//Door latch trigger
int doorLatchLastState = LOW; // Variable to store the last state of the door latch sensor
int doorLatchCurrentState = LOW; // Variable to store the current state of the door latch sensor

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 13, 12); // Initialize Serial2 for HC-12 communication
  Wire.begin(1, 2);

  lightMeter.begin();
  WiFi.begin(ssid, password);  // Connect to Wi-Fi
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  pinMode(buzzerPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP); // Set the button pin as input with internal pull-up resistor
  pinMode(doorLatchSensorPin, INPUT);

  digitalWrite(buzzerPin, LOW); // Ensure buzzer is off initially

  Serial.println("Started");

  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (rtc.lostPower() || !rtc.now().isValid()) {
    Serial.println("RTC lost power, let's set the time!");
    setRTCFromNTP();
  }

  byte version = reg_db(PCBARTISTS_DBM, I2C_REG_VERSION);
  Serial.print("dbMeter VERSION = 0x");
  Serial.println(version, HEX);

  byte id[4];
  id[0] = reg_db(PCBARTISTS_DBM, I2C_REG_ID3);
  id[1] = reg_db(PCBARTISTS_DBM, I2C_REG_ID2);
  id[2] = reg_db(PCBARTISTS_DBM, I2C_REG_ID1);
  id[3] = reg_db(PCBARTISTS_DBM, I2C_REG_ID0);

  dht.begin();
  PM25AQI_init();
}

void loop() {
  //HC12 input check
  if (Serial2.available() > 0) {
    // Read the message from HC-12
    String message = Serial2.readStringUntil('\n');

    // Print the received message to Serial Monitor
    Serial.print("Received: ");
    Serial.println(message);
  }

  //Config RESET button
  buttonState = digitalRead(buttonPin);
  unsigned long currentMillis = millis();
  if (buttonState == LOW && lastButtonState == HIGH) { // Button pressed
    buttonPressTime = millis();
  } else if (buttonState == HIGH && lastButtonState == LOW) { // Button released
    if (millis() - buttonPressTime >= longPressTime) {
      Serial.println("Button long pressed for 3 seconds");
    }
  }

  lastButtonState = buttonState; // Update the last button state

  //Door latch
  doorLatchCurrentState = digitalRead(doorLatchSensorPin);
  if (doorLatchCurrentState != doorLatchLastState) {
    // State has changed
    if (doorLatchCurrentState == HIGH) {
      // Sensor changed from low to high (closed to open)
      Serial.println("Door opened");
    } else {
      // Sensor changed from high to low (open to closed)
      Serial.println("Door closed");
    }

    // Update the last state
    doorLatchLastState = doorLatchCurrentState;
  }

  //Millis for 1 sec
  if (currentMillis - previousMillis >= interval) {
    //time reset
    previousMillis = currentMillis;
    Serial.println("-----*******-------*******--------");
    //lux
    Serial.print("Lux : ");
    Serial.println(luxLevel());

    //rtc
    DateTime now = rtc.now();
    Serial.print("Date: ");
    Serial.print(now.year(), DEC);
    Serial.print('-');
    Serial.print(now.month(), DEC);
    Serial.print('-');
    Serial.print(now.day(), DEC);
    Serial.print(" Time: ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();

    //db sound level
    byte sound_level = reg_db(PCBARTISTS_DBM, I2C_REG_DECIBEL);
    Serial.print("dB : ");
    Serial.println(String(sound_level));

    h = dht.readHumidity();
    // Read temperature as Celsius (the default)
    t = dht.readTemperature();
    if (isnan(h) || isnan(t)) {
      Serial.println("Reading error");
      h = 0.0;
      t = 0.0;
    }
    Serial.print("Temperature : ");
    Serial.println(t);
    Serial.print("RH : ");
    Serial.println(h);

    for (int i = 0; i < 4; i++) {
      moistureValues[i] = analogRead(moisturePins[i]);
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(moistureValues[i]);
    }

    update_pms_data();

    String Data = "{";
    Data += "\"pm1\":" + String(pms25_data[0]) + ",";
    Data += "\"pm25\":" + String(pms25_data[1]) + ",";
    Data += "\"pm10\":" + String(pms25_data[2]) + "";
    Data += "}";

    Serial.println(Data);
  }
}

float luxLevel() {
  return lightMeter.readLightLevel();
}

void beepBuzzer() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(buzzerPin, HIGH); // Turn buzzer on
    delay(500);                    // Wait for 500ms
    digitalWrite(buzzerPin, LOW);  // Turn buzzer off
    delay(500);                    // Wait for 500ms
  }
}

void setRTCFromNTP() {
  timeClient.begin();
  timeClient.update();

  unsigned long epochTime = timeClient.getEpochTime();
  DateTime ntpTime = DateTime(epochTime);

  rtc.adjust(ntpTime);
  Serial.println("RTC time set from NTP");
}

byte reg_db(byte addr, byte reg)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, (uint8_t)1);
  byte data = Wire.read();
  return data;
}
