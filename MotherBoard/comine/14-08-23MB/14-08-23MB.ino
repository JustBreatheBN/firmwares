//General Libraries
#include <Wire.h>
#include <Arduino.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <RTClib.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>

//Sensor Libraries
#include <BH1750.h>
#include "DHT.h"
#include "PMS.h"
#include "Adafruit_VL53L0X.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

//GPIO's
const int buzzerPin = 46;
const int buttonPin = 14; // GPIO pin connected to the push button
const int doorLatchSensorPin = 45;
#define DHTPIN 41
const int moisturePins[4] = {4, 5, 6, 7};
int moistureValues[4] = {0, 0, 0, 0}; // Array to store sensor readings
const int redPin = 38;
const int greenPin = 39;
const int bluePin = 40;

//Object Initialization
BH1750 lightMeter;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800, 60000); // IST offset 19800 seconds (5.5 hours)
RTC_DS3231 rtc;
#define PCBARTISTS_DBM 0x48
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);
HardwareSerial hwSerial(1);  // Use UART1
HardwareSerial rbSerial(2);  // Use UART2
HardwareSerial hc12Serial(3);  // Use UART3
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_MPU6050 mpu;
PMS pms(hwSerial);

//PM25AQI Initialize
#define hw_rs 18
#define hw_tx 17
void PM25AQI_init() {
  hwSerial.begin(9600, SERIAL_8N1, hw_rs, hw_tx);
}

//PMS Data
PMS::DATA pms_data;
int pms25_data[3];
void update_pms_data() {
  if (pms.readUntil(pms_data)) {
    pms25_data[0] = pms_data.PM_AE_UG_1_0;
    pms25_data[1] = pms_data.PM_AE_UG_2_5;
    pms25_data[2] = pms_data.PM_AE_UG_10_0;
  }
}

//Variables
const unsigned long interval = 5000; // Interval in milliseconds (5 seconds)
unsigned long previousMillis = 0;    // Store the last time the message was printed

const unsigned long longPressTime = 3000; // 3 seconds in milliseconds
unsigned long buttonPressTime = 0;
bool buttonState = HIGH;
bool lastButtonState = HIGH;

// Replace with your network credentials
const char* ssid = "iQOO Neo9 Pro";
const char* password = "123456789";


//db sound level Variables
#define PCBARTISTS_DBM 0x48
#define I2C_REG_VERSION      0x00
#define I2C_REG_ID3          0x01
#define I2C_REG_ID2          0x02
#define I2C_REG_ID1          0x03
#define I2C_REG_ID0          0x04
#define I2C_REG_DECIBEL      0x0A

float h, t;
int doorLatchLastState = LOW; // Variable to store the last state of the door latch sensor
int doorLatchCurrentState = LOW; // Variable to store the current state of the door latch sensor

void setup() {
  Serial.begin(115200);
  hc12Serial.begin(9600, SERIAL_8N1, 13, 12); // Initialize Serial3 for HC-12 communication
  rbSerial.begin(9600, SERIAL_8N1, 10, 9);    // Initialize Serial2 for relay board communication
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
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
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

  dht.begin();
  PM25AQI_init();
  lox.begin();
  setColor(0, 0, 0);

  if (!mpu.begin(0x69)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(5);
  mpu.setMotionDetectionDuration(80);
  mpu.setInterruptPinLatch(true);
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
}

void loop() {
  unsigned long currentMillis = millis();

  // HC12 input check
  if (hc12Serial.available() > 0) {
    String message = hc12Serial.readStringUntil('\n');
    Serial.print("HC12 Received: ");
    Serial.println(message);
  }

  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n');
    Serial.print("Sending From Mother Board: ");
    Serial.println(message);
    rbSerial.println(message);
  }

  if (rbSerial.available() > 0) {
    String message = rbSerial.readStringUntil('\n');
    Serial.print("Received From Relay Board: ");
    Serial.println(message);
  }

  if (mpu.getMotionInterruptStatus()) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    Serial.print("AccelX:"); Serial.print(a.acceleration.x); Serial.print(",");
    Serial.print("AccelY:"); Serial.print(a.acceleration.y); Serial.print(",");
    Serial.print("AccelZ:"); Serial.print(a.acceleration.z); Serial.print(", ");
    Serial.print("GyroX:"); Serial.print(g.gyro.x); Serial.print(",");
    Serial.print("GyroY:"); Serial.print(g.gyro.y); Serial.print(",");
    Serial.print("GyroZ:"); Serial.print(g.gyro.z); Serial.println("");
  }

  // Config RESET button
  buttonState = digitalRead(buttonPin);
  if (buttonState == LOW && lastButtonState == HIGH) {
    buttonPressTime = millis();
  } else if (buttonState == HIGH && lastButtonState == LOW) {
    if (millis() - buttonPressTime >= longPressTime) {
      Serial.println("Button long pressed for 3 seconds");
    }
  }

  lastButtonState = buttonState;

  // Door latch
  doorLatchCurrentState = digitalRead(doorLatchSensorPin);
  if (doorLatchCurrentState != doorLatchLastState) {
    if (doorLatchCurrentState == HIGH) {
      Serial.println("Door opened");
    } else {
      Serial.println("Door closed");
    }
    doorLatchLastState = doorLatchCurrentState;
  }

  // Millis for 5 seconds interval
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Get sensor data
    float lux = luxLevel();
    float wl = waterLevel();
    DateTime now = rtc.now();
    byte sound_level = reg_db(PCBARTISTS_DBM, I2C_REG_DECIBEL);
    h = dht.readHumidity();
    t = dht.readTemperature();
    update_pms_data();

    if (isnan(h) || isnan(t)) {
      h = 0.0;
      t = 0.0;
    }

    // Get moisture level readings and map them to 0-100
    int totalMoisture = 0;
    for (int i = 0; i < 4; i++) {
      moistureValues[i] = analogRead(moisturePins[i]);
      moistureValues[i] = map(moistureValues[i], 0, 1024, 0, 100); // Assuming 12-bit ADC
      totalMoisture += moistureValues[i];
    }
    int avgMoisture = totalMoisture / 4;

    // Create a JSON document
    StaticJsonDocument<256> doc;

    doc["moistureLevel"] = avgMoisture;
    doc["waterLevel"] = wl;  // Replace with actual water level sensor data
    doc["pm1"] = pms25_data[0];
    doc["pm2_5"] = pms25_data[1];
    doc["pm10"] = pms25_data[2];
    doc["temperature"] = t;
    doc["humidity"] = h;
    doc["lux"] = lux;
    doc["db"] = sound_level;
    doc["batteryLevel"] = 100;  // Replace with actual battery level data
    doc["amp"] = 4.3;  // Replace with actual amp data
    doc["volt"] = 230.4;  // Replace with actual voltage data

    String dateTime = String(now.year()) + "-" + String(now.month()) + "-" + String(now.day()) + " " + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
    doc["dateTime"] = dateTime;

    // Serialize JSON to string
    String output;
    serializeJson(doc, output);

    // Print JSON string to Serial
    Serial.println(output);

//    // Test distance sensor
//    VL53L0X_RangingMeasurementData_t measure;
//    lox.rangingTest(&measure, false);
//    if (measure.RangeStatus != 4) {
//      //      Serial.print("Distance (CM): ");
//      //      Serial.println(measure.RangeMilliMeter / 10);
//      return map(measure.RangeMilliMeter / 10, 4, 23, 0, 100);
//    } else {
//      //      Serial.println(" out of range ");
//      return 0;
//    }

    setColor(random(0, 255), random(0, 255), random(0, 255));
    rbSerial.println("Hello From Mother Board");
  }
}

float luxLevel() {
  return lightMeter.readLightLevel();
}
float waterLevel(){
  // Test distance sensor
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) {
      //      Serial.print("Distance (CM): ");
      //      Serial.println(measure.RangeMilliMeter / 10);
      return map(measure.RangeMilliMeter / 10, 4, 23, 0, 100);
    } else {
      //      Serial.println(" out of range ");
      return 0;
    }
}
void beepBuzzer() {
  for (int i = 0; i < 1; i++) {
    digitalWrite(buzzerPin, HIGH);
    delay(500);
    digitalWrite(buzzerPin, LOW);
    delay(500);
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

byte reg_db(byte addr, byte reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, (uint8_t)1);
  byte data = Wire.read();
  return data;
}

void setColor(int redValue, int greenValue, int blueValue) {
  analogWrite(redPin, redValue);
  analogWrite(greenPin, greenValue);
  analogWrite(bluePin, blueValue);
}
