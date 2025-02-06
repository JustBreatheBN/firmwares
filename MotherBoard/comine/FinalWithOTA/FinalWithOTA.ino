//General Libraries
#include <Wire.h>
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPUpdate.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <RTClib.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "secrets.h"
#include <Preferences.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <WebServer.h>

//Sensor Libraries
#include <BH1750.h>
#include "DHT.h"
#include "PMS.h"
#include "Adafruit_VL53L0X.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

const char* ssidAP = "JustBreathe-Config";
const char* passwordAP = "12345678";
WebServer server(80);
Preferences preferences;
DNSServer dnsServer;

const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 4, 1);
const char* hostName = "jbconfig";



//Future nvs config variables
//String deviceId;
String THINGNAME;
String SUB_Refresh;
String SUB_Config;
String SUB_Status;
String PUB_md;
String PUB_status;
String PUB_senST;
String PUB_trig;
String PUB_asd;
WiFiClientSecure net;
PubSubClient client(net);

StaticJsonDocument<100> alert;
DynamicJsonDocument receivedData(2084);
DynamicJsonDocument asdData(2084);
StaticJsonDocument<1024> sensorStatus;
StaticJsonDocument<1024> mddoc;
StaticJsonDocument<1024> relayData;
const int MQTT_BUFFER_SIZE = 3000;
//char mqttBuffer[MQTT_BUFFER_SIZE];

void messageHandler(char* topic, byte* payload, unsigned int length);
void connectAWS();
void alertTrigger(String alertT, int val);
//void setColorFromHex(const char* hexColor);
void setColor(int redValue, int greenValue, int blueValue);

//Object Initialization
BH1750 lightMeter;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800, 60000); // IST offset 19800 seconds (5.5 hours)
RTC_DS3231 rtc;
#define PCBARTISTS_DBM 0x48
#define DHTPIN 41
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);
SoftwareSerial hwSerial(18, 17); // Use UART1
SoftwareSerial rbSerial(10, 9); // Use UART2
//HardwareSerial hc12Serial(3);  // Use UART3
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_MPU6050 mpu;
PMS pms(hwSerial);

//PM25AQI Initialize
void PM25AQI_init() {
  hwSerial.begin(9600);
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

//Timer Variables
const unsigned long longPressTime = 10000; // 3 seconds in milliseconds
unsigned long buttonPressTime = 0;
unsigned long currentMillis = millis();
unsigned long previousMillisasd = 0;
unsigned long lastButtonPress = 0;
unsigned long previousMillism = 0;
unsigned long previousMillis = 0; // Tracks the last execution time
unsigned long previousMillismd = 0; // Tracks the last execution time
const unsigned long interval = 1000; // 1-second interval
String dateTime;
//GPIO Configuration
const int redPin = 38;
const int greenPin = 39;
const int bluePin = 40;
const int buttonPin = 14;
const int powSup = 11;
const int earSup = 42;
const int buzzerPin = 46;
const int floatSen = 45;

const int moisturePins[4] = {4, 5, 6, 7};
int moistureValues[4] = {0, 0, 0, 0}; // Array to store sensor readings

bool startupTriggered = false;  // State variable to ensure one-time trigger
int floatLastState = LOW; // Variable to store the last state of the door latch sensor
int floatCurrentState = LOW; // Variable to store the current state of the door latch sensor
String ssid, password;
String deviceId = "";
bool buttonState = true;
bool awsConnected = false;  // Flag to track AWS connection status
bool lastButtonState = true;
int powSupCS = HIGH; // Current state of power supply (default to HIGH)
int powSupLS = HIGH; // Last known state of power supply (default to HIGH)
int earSupCS = HIGH; // Current state of earth supply (default to HIGH)
int earSupLS = HIGH; // Last known state of earth supply (default to HIGH)
bool powSupInit = true; // Flag for initial power supply alert
bool earSupInit = true; // Flag for initial earth supply alert
int currentPowSupStatus;
int currentEarSupStatus;
int asdFreq;
int mdFreq;
int irrDelay;
int irrPin;
int npkDelay;
unsigned long irrMillis = 0;
bool irrActive = false;
unsigned long npkMillis = 0;
unsigned long currentMillisasd = millis();
bool npkActive = false;
String indoorSensorID;  // Default to an empty string
float version;
bool bh1750Found = false;
bool lidarFound = false;
bool dbFound = false;
bool rtcFound = false;
bool vibFound = false;
bool dhtFound = false;
bool pmsFound = false;
//db sound level Variables
#define PCBARTISTS_DBM 0x48
#define I2C_REG_VERSION      0x00
#define I2C_REG_ID3          0x01
#define I2C_REG_ID2          0x02
#define I2C_REG_ID1          0x03
#define I2C_REG_ID0          0x04
#define I2C_REG_DECIBEL      0x0A

float h, t;
DateTime now;
float amp = 0, volt = 0;
float lux;
float wl;
byte sound_level;

const int maxI2CDevices = 127;  // Maximum number of I2C addresses
String i2cDevices[maxI2CDevices]; // Array to store found I2C addresses
int i2cDeviceCount = 0;  // Counter for the number of found I2C devices

void handleRoot() {
  server.send_P(200, "text/html", index_html);
}
void handleWifiConfig() {
  if (server.method() == HTTP_POST) {
    ssid = server.arg("ssid");
    password = server.arg("password");

    // Save Wi-Fi credentials to preferences
    preferences.begin("wifi-config", false);
    preferences.putString("ssid", ssid);
    preferences.putString("password", password);
    preferences.end();

    server.send(200, "application/json", "{\"message\":\"Wi-Fi Configuration Received. Connecting...\"}");

    // Attempt to connect to Wi-Fi
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(ssid.c_str(), password.c_str());
    Serial.println("Connecting to Wi-Fi...");
    int counter = 0;
    while (WiFi.status() != WL_CONNECTED && counter < 30) {
      delay(1000);
      Serial.print(".");
      counter++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nConnected to Wi-Fi");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      //      setColorFromHex("#0000FF");
      setColor(0, 0, 255);
      //      ledTicker.detach(); // Stop blinking
    } else {
      Serial.println("\nFailed to connect to Wi-Fi");
      setColor(255, 0, 0);
      //      setColorFromHex("#FF0000");
    }
  } else {
    server.send(405, "text/plain", "Method Not Allowed");
  }
}
void handleAwsStatus() {
  DynamicJsonDocument json(100);
  json["connected"] = awsConnected;  // Use the actual AWS connection status variable
  String response;
  serializeJson(json, response);
  server.send(200, "application/json", response);
}
void handleSaveDeviceConfig() {
  if (server.method() == HTTP_POST) {
    deviceId = server.arg("device-id");

    // Print received data to serial monitor
    Serial.println("Received Device Configuration:");
    Serial.println("Device ID: " + deviceId);
    preferences.begin("device-config", false);
    preferences.putString("deviceId", deviceId);
    preferences.end();

    // Configure WiFiClientSecure to use the AWS IoT device credentials
    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);

    client.setServer(AWS_IOT_ENDPOINT, 8883);
    client.setCallback(messageHandler);

    THINGNAME = deviceId;
    SUB_Refresh = "Devices/C2M/" + deviceId + "/refresh";
    SUB_Config = "Devices/C2M/" + deviceId + "/config";
    SUB_Status = "Devices/C2M/" + deviceId + "/status";
    PUB_md = "Devices/M2C/" + deviceId + "/stationData";
    PUB_status = "Devices/M2C/" + deviceId + "/status";
    PUB_senST = "Devices/M2C/" + deviceId + "/sensorStatus";
    PUB_trig = "Devices/M2C/" + deviceId + "/triggers";
    PUB_asd = "Devices/M2C/" + deviceId + "/airScopeData";

    // Try to connect to AWS IoT only after the device ID is saved
    connectAWS(); // Attempt to connect to AWS IoT

    if (awsConnected) {
      //      ledTicker.detach(); // Stop blinking
      alertTrigger("binded", 1);  // Trigger restart alert
      setColor(0, 255, 0);
      //      setColorFromHex("#00FF00");
      server.send(200, "application/json", "{\"message\":\"Device Configuration Saved and connected to AWS.\"}");
    } else {
      server.send(200, "application/json", "{\"message\":\"Device Configuration Saved but failed to connect to AWS.\"}");
    }
  } else {
    server.send(405, "text/plain", "Method Not Allowed");
  }
}
void handleResetConfig() {
  Serial.println("Config Reset is pressed");
  preferences.begin("wifi-config", false);
  preferences.clear();
  preferences.end();

  preferences.begin("device-config", false);
  preferences.clear();  // Clear device ID and other device-specific configs
  preferences.end();

  preferences.begin("config", false);
  preferences.clear();  // Clear device ID and other device-specific configs
  preferences.end();

  // Disconnect MQTT client if connected
  if (client.connected()) {
    client.disconnect();
  }

  deviceId = "";
  SUB_Refresh = "";
  THINGNAME = "";
  SUB_Config = "";
  SUB_Status = "";
  PUB_md = "";
  PUB_status = "";
  PUB_senST = "";
  PUB_trig = "";
  PUB_asd = "";
  server.send(200, "application/json", "{\"message\":\"Configuration Reset. Please Restart ESP32.\"}");
  // Disconnect from Wi-Fi and clear credentials
  WiFi.disconnect(true, true);  // Disconnect and erase credentials
  delay(500);
  ESP.restart();  // Restart the ESP32 to apply changes
}
void handleScanNetworks() {
  int n = WiFi.scanNetworks();
  String json = "{\"networks\":[";

  for (int i = 0; i < n; ++i) {
    if (i) json += ",";
    json += "{";
    json += "\"ssid\":\"" + WiFi.SSID(i) + "\",";
    json += "\"rssi\":" + String(WiFi.RSSI(i));
    json += "}";
  }
  json += "]}";
  server.send(200, "application/json", json);
}
void handleWifiStatus() {
  DynamicJsonDocument json(256);
  json["connected"] = (WiFi.status() == WL_CONNECTED);
  if (WiFi.status() == WL_CONNECTED) {
    setColor(0, 255, 0);
    //        setColorFromHex("#00FF00");
    json["ip"] = WiFi.localIP().toString();
  }
  String response;
  serializeJson(json, response);
  server.send(200, "application/json", response);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 13, 12); // Serial3 for HC-12
  rbSerial.begin(9600);    // Initialize Serial2 for relay board communication
  Wire.begin(1, 2);
  pinMode(buttonPin, INPUT_PULLUP); // reset button as pullup
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(powSup, INPUT_PULLUP); // Set pin as input with internal pull-up resistor
  pinMode(earSup, INPUT_PULLUP); // Set pin as input with internal pull-up resistor
  pinMode(buzzerPin, OUTPUT);
  pinMode(floatSen, INPUT);

  preferences.begin("wifi-config", false);
  ssid = preferences.getString("ssid", "");
  password = preferences.getString("password", "");
  preferences.end();

  // Check if the Device is configured with wifi
  if (ssid == "") {
    WiFi.softAP(ssidAP, passwordAP);
    delay(100); // Slight delay to ensure AP is fully started
    dnsServer.start(DNS_PORT, "*", apIP);
    Serial.println("Access Point started");
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());

    if (!MDNS.begin(hostName)) {
      Serial.println("Error starting mDNS");
    } else {
      Serial.printf("mDNS responder started at http://%s.local\n", hostName);
    }

    // Serve the configuration page and handle requests
    server.on("/", handleRoot);
    server.on("/save-config", HTTP_POST, handleWifiConfig);
    server.on("/save-device-config", HTTP_POST, handleSaveDeviceConfig);
    server.on("/reset-config", HTTP_POST, handleResetConfig);
    server.on("/wifi-networks", HTTP_GET, handleScanNetworks);
    server.on("/wifi-status", HTTP_GET, handleWifiStatus);
    server.on("/aws-status", HTTP_GET, handleAwsStatus);
    server.begin();
    Serial.println("HTTP server started");
    setColor(255, 0, 255);
  } else {
    // Configure WiFiClientSecure to use the AWS IoT device credentials
    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);

    client.setServer(AWS_IOT_ENDPOINT, 8883);
    client.setCallback(messageHandler);

    preferences.begin("device-config", false);
    deviceId = preferences.getString("deviceId", "");
    preferences.end();
    if (deviceId != "") {
      Serial.println("Found Device ID alloted to");
      Serial.println(deviceId);
      THINGNAME = deviceId;
      connectAWS();
      SUB_Refresh = "Devices/C2M/" + deviceId + "/refresh";
      SUB_Config = "Devices/C2M/" + deviceId + "/config";
      SUB_Status = "Devices/C2M/" + deviceId + "/status";
      PUB_md = "Devices/M2C/" + deviceId + "/stationData";
      PUB_status = "Devices/M2C/" + deviceId + "/status";
      PUB_senST = "Devices/M2C/" + deviceId + "/sensorStatus";
      PUB_trig = "Devices/M2C/" + deviceId + "/triggers";
      PUB_asd = "Devices/M2C/" + deviceId + "/airScopeData";
      Serial.println("Publshing topics");
      Serial.println(PUB_md);
      Serial.println(PUB_status);
      Serial.println(PUB_senST);
      Serial.println(PUB_trig);
      Serial.println(PUB_asd);
      Serial.println("Subscribed topics");
      Serial.println(SUB_Refresh);
      Serial.println(SUB_Config);
      Serial.println(SUB_Status);
    }
    else {
      Serial.println("Connected to wifi - Device ID Is missing");
    }
    // Attempt to connect to saved Wi-Fi credentials
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), password.c_str());
    //    WiFi.begin(savedSSID.c_str(), savedPassword.c_str());
    Serial.println("Connecting to saved Wi-Fi...");
    int counter = 0;
    while (WiFi.status() != WL_CONNECTED && counter < 20) {
      delay(1000);
      Serial.print(".");
      counter++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nConnected to Wi-Fi");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());

      // Check if internet is available
      if (WiFi.status() == WL_CONNECTED) {
        setColor(0, 255, 0);
        //          setColorFromHex("#00FF00");
        //        // Configure WiFiClientSecure to use the AWS IoT device credentials
        //        net.setCACert(AWS_CERT_CA);
        //        net.setCertificate(AWS_CERT_CRT);
        //        net.setPrivateKey(AWS_CERT_PRIVATE);
        //
        //        client.setServer(AWS_IOT_ENDPOINT, 8883);
        //        client.setCallback(messageHandler);
        //
        //        preferences.begin("device-config", false);
        //        deviceId = preferences.getString("deviceId", "");
        //        preferences.end();
        //        if (deviceId != "") {
        //          Serial.println("Found Device ID alloted to");
        //          Serial.println(deviceId);
        //          THINGNAME = deviceId;
        //          connectAWS();
        //          SUB_Refresh = "Devices/C2M/" + deviceId + "/refresh";
        //          SUB_Config = "Devices/C2M/" + deviceId + "/config";
        //          SUB_Status = "Devices/C2M/" + deviceId + "/status";
        //          PUB_md = "Devices/M2C/" + deviceId + "/stationData";
        //          PUB_status = "Devices/M2C/" + deviceId + "/status";
        //          PUB_senST = "Devices/M2C/" + deviceId + "/sensorStatus";
        //          PUB_trig = "Devices/M2C/" + deviceId + "/triggers";
        //          PUB_asd = "Devices/M2C/" + deviceId + "/airScopeData";
        //          Serial.println("Publshing topics");
        //          Serial.println(PUB_md);
        //          Serial.println(PUB_status);
        //          Serial.println(PUB_senST);
        //          Serial.println(PUB_trig);
        //          Serial.println(PUB_asd);
        //          Serial.println("Subscribed topics");
        //          Serial.println(SUB_Refresh);
        //          Serial.println(SUB_Config);
        //          Serial.println(SUB_Status);
        //        }
        //        else {
        //          Serial.println("Connected to wifi - Device ID Is missing");
        //        }
      } else {
        setColor(255, 0, 0);
        //        setColorFromHex("#FF0000");
      }
    } else {
      Serial.println("\nFailed to connect to saved Wi-Fi");
      //      setColorFromHex("#FF0000");
      setColor(255, 0, 0);
    }
  }
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
  if (pms25_data[0] <= 0 & pms25_data[1] <= 0 & pms25_data[2] <= 0) {
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
    else {
      now = rtc.now();  // Initialize `now` from RTC if valid
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

  loadConfiguration();
}

void loop() {
  currentMillis = millis();
  dnsServer.processNextRequest();
  server.handleClient();

  if (!client.connected()) {
    if (WiFi.status() == WL_CONNECTED) {
      if (deviceId != "" && THINGNAME != "") {
        Serial.println("Aws device reconnecting");
        Serial.println(deviceId);
        connectAWS();
      }
    }
  }
  client.loop();
  // Trigger the "restart" alert only once after AWS connection
  if (!startupTriggered) {
    delay(1000);
    alertTrigger("restart", 1);  // Trigger restart alert
    delay(100);
    senDetails();
    startupTriggered = true;    // Mark as triggered
  }

  // Config RESET button
  buttonState = digitalRead(buttonPin);
  if (buttonState == false && lastButtonState == true) {
    buttonPressTime = millis();
  } else if (buttonState == true && lastButtonState == false) {
    if (millis() - buttonPressTime >= longPressTime) {
      Serial.println("Button long pressed for 10 seconds");
      handleResetConfig();
    }
  }
  lastButtonState = buttonState;
  // Check for incoming JSON commands via Serial.
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove whitespace/newlines
    if (input.length() > 0) {
      Serial.print("Received JSON: ");
      Serial.println(input);

      // Parse the JSON command.
      StaticJsonDocument<256> docT;
      DeserializationError error = deserializeJson(docT, input);
      if (error) {
        Serial.print("JSON parse error: ");
        Serial.println(error.c_str());
        return;
      }

      // Check if the command contains "update": 1.
      if (docT.containsKey("update") && docT["update"] == 1) {
        // Check for the URL in the JSON.
        if (docT.containsKey("url")) {
          const char* url = docT["url"];
          Serial.print("Starting OTA update with URL: ");
          Serial.println(url);

          // Determine whether to use HTTPS or HTTP.
          String urlStr(url);
          // In your OTA update section (e.g., inside your Serial JSON command processing):
          if (urlStr.startsWith("https")) {
            // Create a separate client for OTA updates
            WiFiClientSecure otaClient;

            // For testing, you can set it to insecure (not recommended for production)
            otaClient.setInsecure();

            // Alternatively, for production, set the proper GitHub CA certificate:
            // otaClient.setCACert(githubCACert);

            Serial.println("Using separate WiFiClientSecure (otaClient) for HTTPS update.");
            t_httpUpdate_return ret = httpUpdate.update(otaClient, url);

            switch (ret) {
              case HTTP_UPDATE_FAILED:
                Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n",
                              httpUpdate.getLastError(),
                              httpUpdate.getLastErrorString().c_str());
                break;
              case HTTP_UPDATE_NO_UPDATES:
                Serial.println("HTTP_UPDATE_NO_UPDATES");
                break;
              case HTTP_UPDATE_OK:
                Serial.println("HTTP_UPDATE_OK - Update successful!");
                // The board will restart automatically.
                break;
            }
          } else { // Use standard client for HTTP URLs.
            WiFiClient otaClient;
            Serial.println("Using WiFiClient for HTTP update.");
            t_httpUpdate_return ret = httpUpdate.update(otaClient, url);
            switch (ret) {
              case HTTP_UPDATE_FAILED:
                Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n",
                              httpUpdate.getLastError(),
                              httpUpdate.getLastErrorString().c_str());
                break;
              case HTTP_UPDATE_NO_UPDATES:
                Serial.println("HTTP_UPDATE_NO_UPDATES");
                break;
              case HTTP_UPDATE_OK:
                Serial.println("HTTP_UPDATE_OK - Update successful!");
                // The board will restart automatically.
                break;
            }
          }
        } else {
          Serial.println("Update command received but no URL provided in JSON.");
        }
      } else {
        Serial.println("Invalid command. To trigger an update, send:");
        Serial.println("{\"update\":1, \"url\":\"<your firmware raw URL>\"}");
      }
    }
  }
  //Airscope HC12 redirect
  //  char message[2048];
  if (Serial2.available() > 0) {
    String message;
    message = Serial2.readStringUntil('}');
    message = message + "}";
    Serial.println(message);
    receivedData.clear();
    DeserializationError err = deserializeJson(receivedData, message);
    if (!err) {
      asdData.clear();
      String di = receivedData["i"];
      if (di == indoorSensorID) {
        if (receivedData.containsKey("pm1")) {
          asdData["pm1"] = receivedData["pm1"];
        } else {
          asdData["pm1"] = 0;
        }

        if (receivedData.containsKey("pm2_5")) {
          asdData["pm2_5"] = receivedData["pm2_5"];
        } else {
          asdData["pm2_5"] = 0;
        }

        if (receivedData.containsKey("pm10")) {
          asdData["pm10"] = receivedData["pm10"];
        } else {
          asdData["pm10"] = 0;
        }

        if (receivedData.containsKey("temperature1")) {
          asdData["temperature1"] = receivedData["temperature1"];
        } else {
          asdData["temperature1"] = 0;
        }

        if (receivedData.containsKey("temperature2")) {
          asdData["temperature2"] = receivedData["temperature2"];
        } else {
          asdData["temperature2"] = 0;
        }

        if (receivedData.containsKey("humidity")) {
          asdData["humidity"] = receivedData["humidity"];
        } else {
          asdData["humidity"] = 0;
        }

        if (receivedData.containsKey("lux")) {
          asdData["lux"] = receivedData["lux"];
        } else {
          asdData["lux"] = 0;
        }

        if (receivedData.containsKey("db")) {
          asdData["db"] = receivedData["db"];
        } else {
          asdData["db"] = 0;
        }

        if (receivedData.containsKey("tvoc")) {
          asdData["tvoc"] = receivedData["tvoc"];
        } else {
          asdData["tvoc"] = 0;
        }

        if (receivedData.containsKey("co2")) {
          asdData["co2"] = receivedData["co2"];
        } else {
          asdData["co2"] = 0;
        }

        if (receivedData.containsKey("so2")) {
          asdData.remove("so2");
        }

        if (receivedData.containsKey("o2")) {
          asdData["o2"] = receivedData["o2"];
        } else {
          asdData["o2"] = 0;
        }

        if (receivedData.containsKey("o3")) {
          asdData["o3"] = receivedData["o3"];
        } else {
          asdData["o3"] = 0;
        }

        if (receivedData.containsKey("co")) {
          asdData["co"] = receivedData["co"];
        } else {
          asdData["co"] = 0;
        }

        if (receivedData.containsKey("no2")) {
          asdData["no2"] = receivedData["no2"];
        } else {
          asdData["no2"] = 0;
        }

        if (receivedData.containsKey("nh3")) {
          asdData["nh3"] = receivedData["nh3"];
        } else {
          asdData["nh3"] = 0;
        }

        if (receivedData.containsKey("dateTime")) {
          asdData["dateTime"] = receivedData["dateTime"];
        } else {
          asdData["dateTime"] = String(now.year()) + "-" + String(now.month()) + "-" + String(now.day()) + " " + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
        }

        if (currentMillis - previousMillisasd >= asdFreq * 1000) {
          Serial.println("Deserialization success");
          previousMillisasd = currentMillis;
          publishToMQTT(PUB_asd, asdData.as<JsonObject>());
        }
      }
      else {
        Serial.println("Recieved the HC12 data, but Indoor ID not matching");
        //        Serial.println(asdData);
        serializeJsonPretty(asdData, Serial);
      }
    } else {
      Serial.println("JSON parsing error");
    }
  }

  if (rbSerial.available() > 0) {
    String message = rbSerial.readStringUntil('\n');
    //    Serial.print("Received From Relay Board: ");
    //    Serial.println(message);

    // Parse and process the received JSON data
    relayData.clear();
    DeserializationError err = deserializeJson(relayData, message);
    if (!err) {
      //      relayData["device_id"] = deviceId;
      if (relayData.containsKey("toggle") || relayData.containsKey("regulate")) {
        relayData.remove("9");
        publishToMQTT(PUB_status, relayData.as<JsonObject>());
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

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Read the current states
    powSupCS = digitalRead(powSup);
    earSupCS = digitalRead(earSup);

    // Check for state changes and trigger alerts if necessary
    checkPowerStateChange(powSupCS, powSupLS, "powerSupply", powSupInit);
    checkPowerStateChange(earSupCS, earSupLS, "earthing", earSupInit);
  }

  // Door latch
  floatCurrentState = digitalRead(floatSen);
  if (floatCurrentState != floatLastState) {
    if (floatCurrentState == HIGH) {
      alertTrigger("waterFull", 1);
      beepBuzzer();
    } else {
      alertTrigger("waterFull", 0);
    }
    floatLastState = floatCurrentState;
  }

  if (mpu.getMotionInterruptStatus() && vibFound) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    alertTrigger("hardImpact", 1);
  }
  // Millis for 5 seconds interval
  if (currentMillis - previousMillism >= mdFreq * 60000) { //60000
    previousMillism = currentMillis;

    // Get sensor data
    if (bh1750Found) {
      lux = luxLevel();
    } else {
      lux = 0;
    }
    if (lidarFound) {
      wl = waterLevel();
    }
    else {
      wl = 0;
    }
    if (rtcFound) {
      now = rtc.now();
    } else {
      timeClient.update();
      unsigned long epochTime = timeClient.getEpochTime();
      now = DateTime(epochTime);
    }
    if (dbFound) {
      sound_level = reg_db(PCBARTISTS_DBM, I2C_REG_DECIBEL);
    }
    else {
      sound_level = 0;
    }
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
      if (moistureValues[i] < 670) {
        moistureValues[i] = 100;
      } else if (moistureValues[i] > 870) {
        moistureValues[i] = 0;
      }
      else {
        moistureValues[i] = map(moistureValues[i], 670, 870, 100, 0); // Assuming 12-bit ADC
      }
      totalMoisture += moistureValues[i];
      Serial.print("Moisture Sensor ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(moistureValues[i]);
    }
    int avgMoisture = totalMoisture / 4;
    // Create a JSON document
    mddoc.clear();
    //    mddoc["device_id"] = deviceId;
    mddoc["moistureLevel"] = avgMoisture;
    mddoc["waterLevel"] = wl;  // Replace with actual water level sensor data
    mddoc["pm1"] = pms25_data[0];
    mddoc["pm2_5"] = pms25_data[1];
    mddoc["pm10"] = pms25_data[2];
    mddoc["temperature"] = ((int)(t * 100)) / 100.0;
    mddoc["humidity"] = ((int)(h * 100)) / 100.0;
    mddoc["lux"] = ((int)(lux * 100)) / 100.0;
    mddoc["db"] = sound_level;
    mddoc["battery"] = 100;  // Replace with actual battery level data
    mddoc["amp"] = roundf(amp * 100) / 100.0; // Replace with actual amp data
    mddoc["volt"] = roundf(volt * 100) / 100.0; // Replace with actual voltage data
    dateTime = String(now.year()) + "-" + String(now.month()) + "-" + String(now.day()) + " " + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
    mddoc["dateTime"] = dateTime;
    publishToMQTT(PUB_md, mddoc.as<JsonObject>());
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

float luxLevel() {
  return lightMeter.readLightLevel();
}

float waterLevel() {
  // Test distance sensor
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    int c = measure.RangeMilliMeter / 10;
    Serial.println("Water level CM");
    Serial.println(c);
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

// Function to trigger alerts based on state change
void checkPowerStateChange(int &currentState, int &lastState, const String &supplyType, bool &initFlag) {
  if (initFlag || currentState != lastState) {
    // Trigger alert at startup or when the state changes
    if (currentState == LOW) {
      alertTrigger(supplyType, 1); // State is OK
    } else {
      StaticJsonDocument<256> poweroffRelay;
      //      // Prepare JSON structure for poweroffRelay
      //      poweroffRelay["toggle"]["1"] = 0;
      //      poweroffRelay["toggle"]["2"] = 0;
      //      poweroffRelay["toggle"]["3"] = 0;
      //      poweroffRelay["toggle"]["4"] = 0;
      //      poweroffRelay["toggle"]["5"] = 0;
      //      poweroffRelay["toggle"]["6"] = 0;
      //      poweroffRelay["toggle"]["7"] = 0;
      //      poweroffRelay["toggle"]["8"] = 0;
      //
      //      poweroffRelay["regulate"]["1"] = 0;
      //      poweroffRelay["regulate"]["2"] = 0;
      //
      //      // Serialize and send JSON over Serial
      //      String rjsonString;
      //      serializeJson(poweroffRelay, rjsonString);
      //      Serial.println("Sending power off to relay board, No power Supply");
      //      rbSerial.println(rjsonString);
      //      Serial.println(rjsonString + "\n");
      alertTrigger(supplyType, 0); // State is FAIL
    }

    // Update the last known state
    lastState = currentState;

    // Clear the initialization flag after the first alert
    initFlag = false;
  }
}

void beepBuzzer() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(buzzerPin, HIGH);
    delay(500);
    digitalWrite(buzzerPin, LOW);
    delay(500);
  }
}


void connectAWS() {
  //    Serial.println("Connecting to AWS IoT");
  if (WiFi.status() == WL_CONNECTED) {
    if (THINGNAME != "" && deviceId != "") {
      while (!client.connect(THINGNAME.c_str())) {
        Serial.println(client.state());
        Serial.print(".");
        delay(100);
      }
      if (client.connected()) {

        Serial.println("AWS IoT Connected!");
        client.subscribe(SUB_Config.c_str());
        client.subscribe(SUB_Status.c_str());
        client.subscribe(SUB_Refresh.c_str());
        awsConnected = true;  // Set awsConnected to true only after successful connection
        setColor(0, 255, 0);
        //            setColorFromHex("#00FF00");
      } else {
        Serial.println("AWS IoT Timeout!");
        //        setColor(0, 0, 255);
        //            setColorFromHex("#0000FF");
        awsConnected = false;
      }
    }
  } else {
    Serial.println("WiFi is not connected");
    awsConnected = false;
    setColor(255, 0, 0);
    //        setColorFromHex("#FF0000");
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
  //  Serial.println();

  // Perform logic based on the topic
  if (strcmp(topic, SUB_Config.c_str()) == 0) {
    Serial.println("Processing configuration data...");
    saveConfiguration(doc.as<JsonObject>());  // Save the configuration to NVS
  } else if (strcmp(topic, SUB_Status.c_str()) == 0) {
    Serial.println("Processing relay status data...");
    // Remove "10" from the "toggle" object
    JsonObject toggle = doc["toggle"];
    JsonObject regulate = doc["regulate"];

    if (doc.containsKey("indicator")) {
      Serial.println("Indicator present");
      //      const char* indicatorValue = doc["indicator"];
      //      setColorFromHex(indicatorValue);
    }
    // Load irrDelay and irrPin from NVS
    Preferences preferences;
    if (!preferences.begin("config", true)) {
      Serial.println("Failed to open NVS for reading");
      return;
    }

    irrDelay = preferences.getInt("irrDelay", 120);
    irrPin = preferences.getInt("irrPin", 1);
    npkDelay = preferences.getInt("npkDelay", 5);
    preferences.end();
    if (toggle.containsKey("10") && toggle["10"] == 1) {
      Serial.println("Buzzer present");
      beepBuzzer();
      toggle.remove("10");
    }
    if (toggle.containsKey(String(irrPin)) && toggle[String(irrPin)] == 1) {
      Serial.println("Irrigation present");
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
    if (toggle.containsKey("9")) {
      if (toggle["9"] == 1) {
        Serial.println("NPK present");
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
    }
    if (toggle.size() > 0 || regulate.size() > 0) {
      Serial.println("Rest of the relay present");
      Serial.println("Data to send from Motherboard to relay board");
      delay(500);
      //      serializeJsonPretty(doc, Serial);
      String dd;
      serializeJson(doc, dd);
      rbSerial.print(dd + "\n");
    }
    Serial.println("Out of the status topic");
  } else if (strcmp(topic, SUB_Refresh.c_str()) == 0) {
    Serial.println("Processing Refresh data...");
    publishToMQTT(PUB_asd, asdData.as<JsonObject>());
    delay(500);
    publishToMQTT(PUB_md, mddoc.as<JsonObject>());
    delay(500);
    senDetails();
    delay(500);
  } else {
    Serial.println("Unknown topic. No action taken.");
  }
}

void publishToMQTT(const String& topic, const JsonObject& data) {
  if (deviceId == "" || THINGNAME == "") {
    Serial.println("Device ID or Thing Name is empty. Cannot publish.");
    return;
  }
  char mqttBuffer[MQTT_BUFFER_SIZE];
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      Serial.println("Reconnecting to AWS IoT to publish the data...");
      connectAWS();
    }
    //    data["device_id"] = deviceId;
    serializeJson(data, mqttBuffer, MQTT_BUFFER_SIZE);
    Serial.print("Publishing to topic: ");
    Serial.println(topic);
    Serial.print("Payload: ");
    Serial.println(mqttBuffer);

    if (client.connected()) {
      if (client.publish(topic.c_str(), mqttBuffer)) {
        Serial.println("Data published successfully");
      } else {
        Serial.print("Failed to publish, MQTT state: ");
        Serial.println(client.state());
        Serial.print("Payload size: ");
        Serial.println(strlen(mqttBuffer));
      }
    } else {
      Serial.println("MQTT client not connected");
    }
  } else {
    setColor(255, 0, 0);
    Serial.println("WiFi not connected");
  }
}

void alertTrigger(String alertT, int val) {
  alert.clear();
  //  alert["device_id"] = deviceId;
  alert[alertT] = val;
  serializeJsonPretty(alert, Serial);
  publishToMQTT(PUB_trig, alert.as<JsonObject>());
}

void setColor(int redValue, int greenValue, int blueValue) {
  // Check redValue
  if (redValue == 255) {
    digitalWrite(redPin, HIGH);
  } else {
    digitalWrite(redPin, LOW);
  }

  // Check greenValue
  if (greenValue == 255) {
    digitalWrite(greenPin, HIGH);
  } else {
    digitalWrite(greenPin, LOW);
  }

  // Check blueValue
  if (blueValue == 255) {
    digitalWrite(bluePin, HIGH);
  } else {
    digitalWrite(bluePin, LOW);
  }
}

void saveConfiguration(const JsonObject& config) {
  Preferences preferences;
  if (!preferences.begin("config", false)) {  // Open in read-write mode
    Serial.println("Failed to open NVS for writing");
    return;
  }
  if (config.containsKey("restart") && config["restart"] == 1) {
    ESP.restart();  // Restart the ESP32 to apply changes
  }
  if (config.containsKey("asdFreq")) {
    preferences.putInt("asdFreq", config["asdFreq"]);
  }
  if (config.containsKey("mdFreq")) {
    preferences.putInt("mdFreq", config["mdFreq"]);
  }
  if (config.containsKey("irrDelay")) {
    preferences.putInt("irrDelay", config["irrDelay"]);
  }
  if (config.containsKey("irrPin")) {
    preferences.putInt("irrPin", config["irrPin"]);
  }
  if (config.containsKey("npkDelay")) {
    preferences.putInt("npkDelay", config["npkDelay"]);
  }
  if (config.containsKey("indoorSensorID")) {
    preferences.putString("indoorSensorID", config["indoorSensorID"].as<String>());
  }
  if (config.containsKey("version")) {
    preferences.putFloat("version", config["version"]);
  }
  // Close the preferences as soon as you're done
  preferences.end();
  loadConfiguration();
  Serial.println("Configuration saved to NVS.");
}

void loadConfiguration() {
  Preferences preferences;
  if (!preferences.begin("config", true)) {  // Open in read-only mode
    Serial.println("Failed to open NVS for reading");
    // Assign default values
    asdFreq = 2;
    mdFreq = 1;
    irrDelay = 120;
    irrPin = 1;
    npkDelay = 5;
    indoorSensorID = "";  // Default to an empty string
    version = 1.0;

    // Print loaded configuration
    Serial.println("No NVS - Loading Default Values:");
    Serial.println("asdFreq: " + String(asdFreq));
    Serial.println("mdFreq: " + String(mdFreq));
    Serial.println("irrDelay: " + String(irrDelay));
    Serial.println("irrPin: " + String(irrPin));
    Serial.println("npkDelay: " + String(npkDelay));
    Serial.println("indoorSensorID: " + indoorSensorID);
    Serial.println("version: " + String(version));
    return;
  }

  asdFreq = preferences.getInt("asdFreq", 2);
  mdFreq = preferences.getInt("mdFreq", 1);
  irrDelay = preferences.getInt("irrDelay", 120);
  irrPin = preferences.getInt("irrPin", 1);
  npkDelay = preferences.getInt("npkDelay", 5);
  indoorSensorID = preferences.getString("indoorSensorID", "");
  version = preferences.getFloat("version", 1.0);


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
  Serial.println("version: " + String(version));
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

  sensorStatus.clear();
  //  sensorStatus["device_id"] = deviceId;
  sensorStatus["lux"] = bh1750Found ? 1 : 0;
  sensorStatus["waterLevel"] = lidarFound ? 1 : 0;
  sensorStatus["db"] = dbFound ? 1 : 0;
  sensorStatus["rtc"] = rtcFound ? 1 : 0;
  sensorStatus["vib"] = vibFound ? 1 : 0;
  sensorStatus["moisture"] = 1;
  sensorStatus["relayBoard"] = 1;
  sensorStatus["dht"] = dhtFound ? 1 : 0;
  sensorStatus["pms"] = pmsFound ? 1 : 0;

  serializeJson(sensorStatus, Serial);
  publishToMQTT(PUB_senST, sensorStatus.as<JsonObject>());
}
