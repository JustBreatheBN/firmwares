//General Libraries
#include <Wire.h>
#include <Arduino.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <RTClib.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "secrets.h"
#include <Preferences.h>

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
const int powSup = 11;
const int earSup = 42;

//Future nvs config variables
String deviceID = "DemoKit";
// Replace with your network credentials
const char* ssid = "iQOO Neo9 Pro";
const char* password = "123456789";
// Define MQTT topics
String SUB_Config = "Devices/C2M/" + deviceID + "/config";
String SUB_Status = "Devices/C2M/" + deviceID + "/status";
String PUB_md = "Devices/M2C/" + deviceID + "/stationData";
String PUB_status = "Devices/M2C/" + deviceID + "/status";
String PUB_senST = "Devices/M2C/" + deviceID + "/sensorStatus";
String PUB_trig = "Devices/M2C/" + deviceID + "/triggers";
WiFiClientSecure net;
PubSubClient client(net);
Preferences preferences;
// Increase MQTT buffer size
const int MQTT_BUFFER_SIZE = 1024;
char mqttBuffer[MQTT_BUFFER_SIZE];

// Function prototypes
void messageHandler(char* topic, byte* payload, unsigned int length);
//void sendStationData();
//void sendRelayData();
//bool checkRelayWorking(int relayIndex);
void connectAWS();

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
const unsigned long interval = 60000; // Interval in milliseconds (5 seconds)
unsigned long previousMillis = 0;    // Store the last time the message was printed

const unsigned long longPressTime = 3000; // 3 seconds in milliseconds
unsigned long buttonPressTime = 0;
bool buttonState = HIGH;
bool lastButtonState = HIGH;

bool bh1750Found;
bool lidarFound;
bool dbFound;
bool rtcFound;
bool vibFound;
bool dhtFound;
bool pmsFound;

const int maxI2CDevices = 127;  // Maximum number of I2C addresses
String i2cDevices[maxI2CDevices]; // Array to store found I2C addresses
int i2cDeviceCount = 0;  // Counter for the number of found I2C devices

//// Replace with your network credentials
//const char* ssid = "Just Breathe";
//const char* password = "9845055994";


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

int powSupLS = LOW;
int powSupCS = LOW;

int earSupLS = LOW;
int earSupCS = LOW;

unsigned long irrMillis = 0;
bool irrActive = false;
int irrPin = 0;
int irrDelay = 0;
unsigned long npkMillis = 0;
bool npkActive = false;
int npkPin = 9;  // Assuming pin 9 is fixed for this logic
int npkDelay = 0;

int mdFreq = 1;
float amp = 0, volt = 0;
void setup() {
  Serial.begin(115200);
  hc12Serial.begin(9600, SERIAL_8N1, 13, 12); // Initialize Serial3 for HC-12 communication
  rbSerial.begin(9600, SERIAL_8N1, 10, 9);    // Initialize Serial2 for relay board communication
  Wire.begin(1, 2);

  // Initialize WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);  // Connect to Wi-Fi
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println("Wifi COnnected");
  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  client.setServer(AWS_IOT_ENDPOINT, 8883);
  client.setCallback(messageHandler);

  connectAWS();

  pinMode(buzzerPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP); // Set the button pin as input with internal pull-up resistor
  pinMode(doorLatchSensorPin, INPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(powSup, INPUT_PULLUP);
  pinMode(earSup, INPUT_PULLUP);
  digitalWrite(buzzerPin, LOW); // Ensure buzzer is off initially

  Serial.println("Started");

  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());

  scanI2CDevices();
  dht.begin();
  h = dht.readHumidity();
  t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    h = 0.0;
    t = 0.0;
    dhtFound = false;
  }
  else {
    dhtFound = true;
  }
  PM25AQI_init();
  update_pms_data();
  if (pms25_data[0] == 0 & pms25_data[1] & pms25_data[2]) {
    pmsFound = false;
  }
  else {
    pmsFound = true;
  }
  senDetails();

  if (bh1750Found) {
    lightMeter.begin();
  }

  if (rtcFound) {
    rtc.begin();
    if (rtc.lostPower() || !rtc.now().isValid()) {
      setRTCFromNTP();
    }
  }
  if (dbFound) {
    byte version = reg_db(PCBARTISTS_DBM, I2C_REG_VERSION);
    byte id[4];
    id[0] = reg_db(PCBARTISTS_DBM, I2C_REG_ID3);
    id[1] = reg_db(PCBARTISTS_DBM, I2C_REG_ID2);
    id[2] = reg_db(PCBARTISTS_DBM, I2C_REG_ID1);
    id[3] = reg_db(PCBARTISTS_DBM, I2C_REG_ID0);
  }

  if (lidarFound) {
    lox.begin();
  }
  //  setColor(0, 0, 0);
  if (vibFound) {
    mpu.begin(0x69);
    delay(10);
    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(5);
    mpu.setMotionDetectionDuration(80);
    mpu.setInterruptPinLatch(true);
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);
  }

  // Load configuration from NVS
  loadConfiguration();
  alertTrigger("restart",1);
}

void loop() {
  if (!client.connected()) {
    connectAWS();
  }
  client.loop();
  unsigned long currentMillis = millis();

  // HC12 input check
  if (hc12Serial.available() > 0) {
    String message = hc12Serial.readStringUntil('\n');
    Serial.print("HC12 Received: ");
    Serial.println(message);

    // Parse and process the received JSON data
    StaticJsonDocument<400> receivedData;
    DeserializationError err = deserializeJson(receivedData, message);
    if (!err) {
      float valueC = receivedData["c"];
      float valueT = receivedData["t"];
      Serial.println(message);
      // Process other data here
    } else {
      Serial.println("JSON parsing error");
    }
  }
  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n');
    if (message == "status") {
      senDetails();
    }
    else if (message == "config") {
      loadConfiguration();
    }
    else {
      //      Serial.println(message);
      rbSerial.println(message + "\n");
    }
    //    Serial.print("Sending From Mother Board: ");
    //    Serial.println(message);
    //    rbSerial.println(message);
  }

  if (rbSerial.available() > 0) {
    String message = rbSerial.readStringUntil('\n');
    //    Serial.print("Received From Relay Board: ");
    Serial.println(message);

    // Parse and process the received JSON data
    StaticJsonDocument<400> relayData;
    DeserializationError err = deserializeJson(relayData, message);
    if (!err) {
      if (relayData.containsKey("toggle") || relayData.containsKey("regulate")) {
        serializeJson(relayData, mqttBuffer, MQTT_BUFFER_SIZE);
        Serial.print("Publishing relay feedback data: ");
        Serial.println(mqttBuffer);
        bool success = client.publish(PUB_status.c_str(), mqttBuffer);
        if (success) {
          Serial.println("relay feedback data published successfully");
        } else {
          Serial.print("Failed to publish relayfeedback data, MQTT state: ");
          Serial.println(client.state());
          Serial.print("Payload size: ");
          Serial.println(strlen(mqttBuffer));
        }
      }
      if (relayData.containsKey("power")) {
        JsonObject powerData = relayData["power"];  // Access the "power" object
        amp = powerData["current_A"];  // Extract "current_A" from "power"
        volt = powerData["voltage_V"];  // Extract "voltage_V" from "power"
      }
      if (relayData.containsKey("isWorking")) {
        int isWorking = relayData["isWorking"];
        Serial.print("isWorking: ");
        Serial.println(isWorking);
      }
    }
  }

  if (mpu.getMotionInterruptStatus()) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    alertTrigger("hardImpact",1);
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
      alertTrigger("doorOpen",1);
//      Serial.println("Door opened");
    } else {
      alertTrigger("doorOpen",0);
//      Serial.println("Door closed");
    }
    doorLatchLastState = doorLatchCurrentState;
  }

powSupCS = digitalRead(powSup);
if(powSupCS != powSupLS){
  if(powSupCS == HIGH){
    alertTrigger("powerSupply",1);
  }else{
    alertTrigger("powerSupply",0);
  }
  powSupLS = powSupCS;
}

earSupCS = digitalRead(earSup);
if(earSupCS != earSupLS){
  if(earSupCS == HIGH){
    alertTrigger("earthSupply",1);
  }else{
    alertTrigger("earthSupply",0);
  }
  earSupLS = earSupCS;
}

  // Millis for 5 seconds interval
  if (currentMillis - previousMillis >= mdFreq * 60000) {
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
      moistureValues[i] = map(moistureValues[i], 0, 1050, 100, 0); // Assuming 12-bit ADC
      totalMoisture += moistureValues[i];
    }
    int avgMoisture = totalMoisture / 4;

    // Create a JSON document
    StaticJsonDocument<1024> doc;

    doc["moisture"] = avgMoisture;
    doc["waterLevel"] = wl;  // Replace with actual water level sensor data
    doc["pm1"] = pms25_data[0];
    doc["pm2_5"] = pms25_data[1];
    doc["pm10"] = pms25_data[2];
    doc["temperature"] = t;
    doc["humidity"] = h;
    doc["lux"] = lux;
    doc["db"] = sound_level;
    doc["battery"] = 100;  // Replace with actual battery level data
    doc["amp"] = amp;  // Replace with actual amp data
    doc["volt"] = volt;  // Replace with actual voltage data

    String dateTime = String(now.year()) + "-" + String(now.month()) + "-" + String(now.day()) + " " + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
    doc["dateTime"] = dateTime;

    serializeJsonPretty(doc, Serial);
    serializeJson(doc, mqttBuffer, MQTT_BUFFER_SIZE);
    Serial.print("Publishing station data: ");
    Serial.println(mqttBuffer);
    bool success = client.publish(PUB_md.c_str(), mqttBuffer);
    if (success) {
      Serial.println("Station data published successfully");
    } else {
      Serial.print("Failed to publish station data, MQTT state: ");
      Serial.println(client.state());
      Serial.print("Payload size: ");
      Serial.println(strlen(mqttBuffer));
    }


    //    setColor(random(0, 255), random(0, 255), random(0, 255));
    //    rbSerial.println("Hello From Mother Board");
  }
  if (irrActive) {
    // Check if the delay period has passed
    if (millis() - irrMillis >= irrDelay * 1000) {
      // Print {"toggle": {"irrPin": 0}}
      StaticJsonDocument<64> toggleDoc;
      toggleDoc["toggle"][String(irrPin)] = 0;

      String toggleOffJson;
      serializeJson(toggleDoc, toggleOffJson);
      rbSerial.println(toggleOffJson + "\n");

      Serial.println("Toggle OFF command sent to rbSerial after delay:");
      Serial.println(toggleOffJson);

      // Reset the state
      irrActive = false;
    }
  }
  // Handle the logic for npkPin (pin 9)
  if (npkActive) {
    // Check if the delay period has passed
    if (millis() - npkMillis >= npkDelay * 1000) {
      // Print {"toggle": {"9": 0}}
      StaticJsonDocument<64> toggleDoc;
      toggleDoc["toggle"]["9"] = 0;

      String toggleOffJson;
      serializeJson(toggleDoc, toggleOffJson);
      rbSerial.println(toggleOffJson + "\n");

      Serial.println("Toggle OFF command for pin 9 sent to rbSerial after delay:");
      Serial.println(toggleOffJson);

      // Reset the state
      npkActive = false;
    }
  }
}

// Connect to AWS IoT
void connectAWS() {
  Serial.println("Connecting to AWS IoT");
  while (!client.connect(THINGNAME)) {
    Serial.print(".");
    delay(100);
  }
  if (client.connected()) {
    Serial.println("AWS IoT Connected!");
    // Subscribe to multiple topics
    client.subscribe(SUB_Config.c_str());
    Serial.print("Subscribed to topic: ");
    Serial.println(SUB_Config);

    client.subscribe(SUB_Status.c_str());
    Serial.print("Subscribed to topic: ");
    Serial.println(SUB_Status);
  } else {
    Serial.println("AWS IoT Timeout!");
  }
}

// Message handler for incoming MQTT messages
void messageHandler(char* topic, byte* payload, unsigned int length) {
  Serial.print("Incoming message from topic: ");
  Serial.println(topic);

  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  // Print the deserialized JSON
  Serial.print("Received JSON: ");
  serializeJsonPretty(doc, Serial);
  Serial.println();

  // Perform logic based on the topic
  if (strcmp(topic, SUB_Config.c_str()) == 0) {
    Serial.println("Processing configuration data...");
    saveConfiguration(doc.as<JsonObject>());  // Save the configuration to NVS
  } else if (strcmp(topic, SUB_Status.c_str()) == 0) {
    Serial.println("Processing status data...");
    // Remove "10" from the "toggle" object
    JsonObject toggle = doc["toggle"];
    JsonObject regulate = doc["regulate"];

    if (doc.containsKey("indicator")) {
      const char* indicatorValue = doc["indicator"];
      setColorFromHex(indicatorValue);
    }
    // Load irrDelay and irrPin from NVS
    Preferences preferences;
    if (!preferences.begin("config", true)) {
      Serial.println("Failed to open NVS for reading");
      return;
    }

    irrDelay = preferences.getInt("irrDelay", 2);
    irrPin = preferences.getInt("irrPin", 0);
    npkDelay = preferences.getInt("npkDelay", 2);
    preferences.end();
    if (toggle.containsKey("10") && toggle["10"] == 1) {
      beepBuzzer();
      toggle.remove("10");
    }
    if (toggle.containsKey(String(irrPin)) && toggle[String(irrPin)] == 1) {
      // Print {"toggle": {"irrPin": 1}}
      StaticJsonDocument<64> toggleDoc;
      toggleDoc["toggle"][String(irrPin)] = 1;

      String toggleOnJson;
      serializeJson(toggleDoc, toggleOnJson);
      rbSerial.println(toggleOnJson + "\n");

      Serial.println("Toggle ON command sent to rbSerial:");
      Serial.println(toggleOnJson);

      // Start the delay period without blocking
      irrMillis = millis();
      irrActive = true;
      toggle.remove(String(irrPin));
    }
    if (toggle.containsKey("9") && toggle["9"] == 1) {
      // Print {"toggle": {"9": 1}}
      StaticJsonDocument<64> toggleDoc;
      toggleDoc["toggle"]["9"] = 1;

      String toggleOnJson;
      serializeJson(toggleDoc, toggleOnJson);
      rbSerial.println(toggleOnJson + "\n");

      Serial.println("Toggle ON command for pin 9 sent to rbSerial:");
      Serial.println(toggleOnJson);

      // Start the delay period without blocking
      npkMillis = millis();
      npkActive = true;
      toggle.remove("9");
    }
    if (toggle.size() > 0 || regulate.size() > 0) {
      delay(500);
      serializeJsonPretty(doc, Serial);
      serializeJson(doc, rbSerial);
    }

  } else {
    Serial.println("Unknown topic. No action taken.");
  }
}

float luxLevel() {
  return lightMeter.readLightLevel();
}

void alertTrigger(String alertT, int val){
  //    Serial.println("Impact Detected");
    StaticJsonDocument<30> alert;
    alert[alertT] = val;
    serializeJsonPretty(alert, Serial);
    serializeJson(alert, mqttBuffer, MQTT_BUFFER_SIZE);
    Serial.print("alert publish: ");
    Serial.println(mqttBuffer);
    bool success = client.publish(PUB_trig.c_str(), mqttBuffer);
    if (success) {
      Serial.println("alert publish Success");
    } else {
      Serial.print("alert publish failed");
      Serial.println(client.state());
      Serial.print("Payload size: ");
      Serial.println(strlen(mqttBuffer));
    }
}

float waterLevel() {
  // Test distance sensor
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    //      Serial.print("Distance (CM): ");
    //      Serial.println(measure.RangeMilliMeter / 10);
    int c = measure.RangeMilliMeter / 10;
    if (c < 4) {
      return 100;
    }
    else if (c > 24) {
      return 0;
    }
    else {
      return map(c, 4, 24, 100, 0);
    }
  } else {
    //      Serial.println(" out of range ");
    return 0;
  }
}

void beepBuzzer() {
  for (int i = 0; i < 3; i++) {
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

void setColorFromHex(const char* hexColor) {
  // Remove the '#' character if present
  if (hexColor[0] == '#') {
    hexColor++;
  }

  // Convert hex strings to integer values
  long number = strtol(hexColor, NULL, 16);

  int redValue = (number >> 16) & 0xFF;
  int greenValue = (number >> 8) & 0xFF;
  int blueValue = number & 0xFF;

  // Use the setColor function to set the RGB LED
  setColor(redValue, greenValue, blueValue);
}

void setColor(int redValue, int greenValue, int blueValue) {
  analogWrite(redPin, redValue);
  analogWrite(greenPin, greenValue);
  analogWrite(bluePin, blueValue);
}

// Function to check if an address is in the array
bool isAddressInArray(String addressToCheck) {
  for (int i = 0; i < i2cDeviceCount; i++) {
    if (i2cDevices[i] == addressToCheck) {
      return true;
    }
  }
  return false;
}

// Function to scan for I2C devices and populate the array
void scanI2CDevices() {
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      String hexAddress = "0x" + String(address, HEX); // Convert to hexadecimal
      i2cDevices[i2cDeviceCount] = hexAddress;
      i2cDeviceCount++;
    }
  }
}

void senDetails() {
  // Create a JSON object to store the sensor statuses
  i2cDeviceCount = 0;
  scanI2CDevices();

  bh1750Found = isAddressInArray("0x23");
  lidarFound = isAddressInArray("0x29");
  dbFound = isAddressInArray("0x48");
  rtcFound = isAddressInArray("0x68");
  vibFound = isAddressInArray("0x69");

  //  VL53L0XFound = isAddressInArray("0x29");
  StaticJsonDocument<200> sensorStatus;
  sensorStatus["lux"] = bh1750Found ? 1 : 0;
  sensorStatus["lidar"] = lidarFound ? 1 : 0;
  sensorStatus["db"] = dbFound ? 1 : 0;
  sensorStatus["rtc"] = rtcFound ? 1 : 0;
  sensorStatus["vib"] = vibFound ? 1 : 0;

  sensorStatus["dht"] = dhtFound ? 1 : 0;
  sensorStatus["pms"] = pmsFound ? 1 : 0;
  //  sensorStatus["vl53l0x"] = VL53L0XFound ? 1 : 0;

  // Serialize the JSON object and send it over the serial port
  serializeJsonPretty(sensorStatus, Serial);
  Serial.println();
  serializeJson(sensorStatus, mqttBuffer, MQTT_BUFFER_SIZE);
  Serial.print("Publishing station data: ");
  Serial.println(mqttBuffer);
  bool success = client.publish(PUB_senST.c_str(), mqttBuffer);
  if (success) {
    Serial.println("Station data published successfully");
  } else {
    Serial.print("Failed to publish station data, MQTT state: ");
    Serial.println(client.state());
    Serial.print("Payload size: ");
    Serial.println(strlen(mqttBuffer));
  }
}

void saveConfiguration(const JsonObject& config) {
  Preferences preferences;
  if (!preferences.begin("config", false)) {  // Open in read-write mode
    Serial.println("Failed to open NVS for writing");
    return;
  }

  preferences.putInt("asdFreq", config["asdFreq"]);
  preferences.putInt("mdFreq", config["mdFreq"]);
  preferences.putInt("irrDelay", config["irrDelay"]);
  preferences.putInt("irrPin", config["irrPin"]);
  preferences.putInt("npkDelay", config["npkDelay"]);
  preferences.putString("indoorSensorID", config["indoorSensorID"].as<String>());
  preferences.putString("outSensorID", config["outSensorID"].as<String>());
  preferences.putFloat("version", config["version"]);
  preferences.putInt("update", config["update"]);

  // Close the preferences as soon as you're done
  preferences.end();

  Serial.println("Configuration saved to NVS.");
}


void loadConfiguration() {
  Preferences preferences;
  if (!preferences.begin("config", true)) {  // Open in read-only mode
    Serial.println("Failed to open NVS for reading");
    return;
  }

  int asdFreq = preferences.getInt("asdFreq", 0);
  mdFreq = preferences.getInt("mdFreq", 1);
  irrDelay = preferences.getInt("irrDelay", 0);
  irrPin = preferences.getInt("irrPin", 0);
  npkDelay = preferences.getInt("npkDelay", 0);
  String indoorSensorID = preferences.getString("indoorSensorID", "");
  String outSensorID = preferences.getString("outSensorID", "");
  float version = preferences.getFloat("version", 0.0);
  int update = preferences.getInt("update", 0);

  // Close the preferences as soon as you're done
  preferences.end();

  // Print loaded configuration
  Serial.println("Loaded configuration from NVS:");
  Serial.println("asdFreq: " + String(asdFreq));
  Serial.println("mdFreq: " + String(mdFreq));
  Serial.println("irrDelay: " + String(irrDelay));
  Serial.println("irrPin: " + String(irrPin));
  Serial.println("npkDelay: " + String(npkDelay));
  Serial.println("indoorSensorID: " + indoorSensorID);
  Serial.println("outSensorID: " + outSensorID);
  Serial.println("version: " + String(version));
  Serial.println("update: " + String(update));
}
