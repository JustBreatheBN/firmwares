//General Libraries
#include <Wire.h>
#include <Arduino.h>
#include <WiFi.h>
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
#include <Ticker.h>
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

Ticker ledTicker;
bool ledState = false;
unsigned long lastButtonPress = 0;

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
//String deviceId;
String THINGNAME;
String SUB_Config;
String SUB_Status;
String PUB_md;
String PUB_status;
String PUB_senST;
String PUB_trig;
String PUB_asd;
WiFiClientSecure net;
PubSubClient client(net);
//Preferences preferences;
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

//Variables
const unsigned long interval = 10000; // Interval in milliseconds (5 seconds)
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

int powSupCS = HIGH; // Current state of power supply (default to HIGH)
int powSupLS = HIGH; // Last known state of power supply (default to HIGH)

int earSupCS = HIGH; // Current state of earth supply (default to HIGH)
int earSupLS = HIGH; // Last known state of earth supply (default to HIGH)

float lux;
float wl;
byte sound_level;

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
String ssid, password;
String deviceId = "";

DateTime now;
bool awsConnected = false;  // Flag to track AWS connection status
const char index_html[] PROGMEM = R"rawliteral(

  <!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Just Breathe Configuration</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            display: flex;
            justify-content: center;
            align-items: center;
            height: 100vh;
            background-color: #f0f0f0;
            margin: 0;
            flex-direction: column;
        }
        .container {
            background-color: #fff;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 0 10px rgba(0, 0, 0, 0.1);
            max-width: 400px;
            width: calc(100% - 40px);
            box-sizing: border-box;
            margin-bottom: 20px;
        }
        h2 {
            margin-bottom: 20px;
            font-size: 24px;
            text-align: center;
        }
        label {
            display: block;
            margin-bottom: 5px;
            font-weight: bold;
        }
        input[type="text"], input[type="password"], textarea, select {
            width: 100%;
            padding: 10px;
            margin-bottom: 20px;
            border: 1px solid #ccc;
            border-radius: 5px;
            box-sizing: border-box;
        }
        input[type="submit"], input[type="button"], input[type="button"].connect-btn {
            background-color: #007bff;
            color: white;
            padding: 10px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            width: 100%;
            font-size: 16px;
            box-sizing: border-box;
            margin-bottom: 10px;
        }
        input[type="submit"]:hover, input[type="button"]:hover {
            background-color: #0056b3;
        }
        .status {
            margin-top: 10px;
            text-align: center;
            font-size: 18px;
        }
        .disabled {
            opacity: 0.6;
            pointer-events: none;
        }
    </style>
</head>
<body>
    <div class="container" id="wifi-card">
        <h2>Wi-Fi Configuration</h2>
        <form id="wifi-form">
            <label for="ssid">Select Wi-Fi Network:</label>
            <select id="ssid" name="ssid"></select>
            <label for="password">Wi-Fi Password:</label>
            <input type="password" id="password" name="password">
            <div id="wifi-status" class="status">Wi-Fi Status: Not Connected</div>
            <input type="button" value="Connect to Wi-Fi" class="connect-btn" onclick="connectToWiFi()">
        </form>
    </div>
//    <div class="container disabled" id="config-card">
    <div class="container" id="config-card">
        <h2>Device Configuration</h2>
        <form id="config-form">
            <label for="device-id">Device ID:</label>
            <input type="text" id="device-id" name="device-id">
            <input type="submit" value="Save Configuration">
        </form>
        <input type="button" value="Reset Configuration" onclick="resetConfig()">
    </div>
    <script>
        document.addEventListener('DOMContentLoaded', function() {
            fetch('/wifi-networks')
                .then(response => response.json())
                .then(data => {
                    const ssidSelect = document.getElementById('ssid');
                    data.networks.forEach(network => {
                        const option = document.createElement('option');
                        option.value = network.ssid;
                        option.textContent = `${network.ssid} (${network.rssi} dBm)`;
                        ssidSelect.appendChild(option);
                    });
                });
        });

        function connectToWiFi() {
            const formData = new FormData(document.getElementById('wifi-form'));

            fetch('/save-config', {
                method: 'POST',
                body: formData
            })
            .then(response => response.json())
            .then(data => {
                alert(data.message);
                // Start checking Wi-Fi connection status
                checkWifiStatus();
            })
            .catch(error => {
                console.error('Error:', error);
                alert('Failed to connect to Wi-Fi.');
            });
        }

        function checkWifiStatus() {
            fetch('/wifi-status')
                .then(response => response.json())
                .then(data => {
                    const wifiStatusDiv = document.getElementById('wifi-status');
                    if (data.connected) {
                        wifiStatusDiv.textContent = `Wi-Fi Status: Connected (IP: ${data.ip})`;
                        enableConfigCard();
                        fetchDevices();
                    } else {
                        wifiStatusDiv.textContent = "Wi-Fi Status: Not Connected";
                        setTimeout(checkWifiStatus, 5000); // Retry after 5 seconds
                    }
                });
        }

//        function enableConfigCard() {
//            const configCard = document.getElementById('config-card');
//            configCard.classList.remove('disabled');
//        }

        function fetchDevices() {
            // Fetch devices or any required data from API once connected to Wi-Fi
            fetch('/fetch-devices')
                .then(response => response.json())
                .then(data => {
                    // Handle device data here (populate a select box or any other UI element if needed)
                })
                .catch(error => {
                    console.error('Error fetching devices:', error);
                });
        }

        document.getElementById('config-form').addEventListener('submit', function(event) {
            event.preventDefault();
            const formData = new FormData(this);

            fetch('/save-device-config', {
                method: 'POST',
                body: formData
            })
            .then(response => response.json())
            .then(data => {
                alert(data.message);
            })
            .catch(error => {
                console.error('Error:', error);
                alert('Failed to save device configuration.');
            });
        });

        function resetConfig() {
            fetch('/reset-config', { method: 'POST' })
                .then(response => response.json())
                .then(data => {
                    alert(data.message);
                })
                .catch(error => {
                    console.error('Error:', error);
                    alert('Failed to reset configuration.');
                });
        }
    </script>
</body>
</html>

)rawliteral";

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
    WiFi.mode(WIFI_STA);
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
      setColorFromHex("#0000FF");
      ledTicker.detach(); // Stop blinking
    } else {
      Serial.println("\nFailed to connect to Wi-Fi");
      setColorFromHex("#FF0000");
    }
  } else {
    server.send(405, "text/plain", "Method Not Allowed");
  }
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
    //Future nvs config variables

    assignedTopics();
    
    server.send(200, "application/json", "{\"message\":\"Device Configuration Saved.\"}");
  } else {
    server.send(405, "text/plain", "Method Not Allowed");
  }
}

void handleResetConfig() {
  preferences.begin("wifi-config", false);
  preferences.clear();
  preferences.end();
  clearTopics();
  server.send(200, "application/json", "{\"message\":\"Configuration Reset. Please Restart ESP32.\"}");
  Serial.println("Configuration Reset. Please Restart ESP32.");
  ESP.restart();
}
void clearTopics(){
  deviceId = "";
  THINGNAME = deviceId;
    SUB_Config = "";
    SUB_Status = "";
    PUB_md = "";
    PUB_status = "";
    PUB_senST = "";
    PUB_trig = "";
    PUB_asd = "";
}
void assignedTopics(){
    THINGNAME = deviceId;
    SUB_Config = "Devices/C2M/" + deviceId + "/config";
    SUB_Status = "Devices/C2M/" + deviceId + "/status";
    PUB_md = "Devices/M2C/" + deviceId + "/stationData";
    PUB_status = "Devices/M2C/" + deviceId + "/status";
    PUB_senST = "Devices/M2C/" + deviceId + "/sensorStatus";
    PUB_trig = "Devices/M2C/" + deviceId + "/triggers";
    PUB_asd = "Devices/M2C/" + deviceId + "/airScopeData";
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
    json["ip"] = WiFi.localIP().toString();
  }
  String response;
  serializeJson(json, response);
  server.send(200, "application/json", response);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 13, 12); // Initialize Serial3 for HC-12 communication
  rbSerial.begin(9600);    // Initialize Serial2 for relay board communication
  Wire.begin(1, 2);
  // Initialize NVS
  preferences.begin("wifi-config", false);
  String savedSSID = preferences.getString("ssid", "");
  String savedPassword = preferences.getString("password", "");
  preferences.end();

  // Start access point mode if no saved Wi-Fi credentials
  if (savedSSID == "") {
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
    server.begin();
    Serial.println("HTTP server started");

    // Start blinking red LED
    ledTicker.attach(0.5, []() {
      ledState = !ledState;
      digitalWrite(redPin, ledState);
    });
  } else {
    // Attempt to connect to saved Wi-Fi credentials
    WiFi.mode(WIFI_STA);
    WiFi.begin(savedSSID.c_str(), savedPassword.c_str());
    Serial.println("Connecting to saved Wi-Fi...");
    int counter = 0;
    while (WiFi.status() != WL_CONNECTED && counter < 5) {
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
        setColorFromHex("#0000FF");
      } else {
        setColorFromHex("#00FF00");
      }
    } else {
      Serial.println("\nFailed to connect to saved Wi-Fi");
      setColorFromHex("#FF0000");
    }
  }
  Serial.println("Wifi COnnected");
  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  client.setServer(AWS_IOT_ENDPOINT, 8883);
  client.setCallback(messageHandler);
  
  preferences.begin("device-config", false);
  deviceId = preferences.getString("deviceId", "");
  preferences.end();
  assignedTopics();
  
  if(deviceId != ""){
    if(WiFi.status() == WL_CONNECTED){
      Serial.println(deviceId);
      connectAWS();
    }
  }
  pinMode(buzzerPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP); // Set the button pin as input with internal pull-up resistor
  pinMode(doorLatchSensorPin, INPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(powSup, INPUT_PULLUP); // Set pin as input with internal pull-up resistor
  pinMode(earSup, INPUT_PULLUP); // Set pin as input with internal pull-up resistor

  powSupCS = digitalRead(powSup); // Read current power supply state
  powSupLS = powSupCS;            // Initialize last state to match current state
  earSupCS = digitalRead(earSup); // Read current earth supply state
  earSupLS = earSupCS;            // Initialize last state to match current state
  digitalWrite(buzzerPin, LOW); // Ensure buzzer is off initially

  checkPowerStateChange(powSupCS, powSupLS, "powerSupply");
  checkPowerStateChange(earSupCS, earSupLS, "earthSupply");
  
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
    else{
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
    mpu.setMotionDetectionThreshold(10);
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
//  Serial.println("Loop started");
  dnsServer.processNextRequest();
  server.handleClient();
  // Only attempt to connect to AWS if not already connected
  if (!client.connected()) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println(deviceId);
      connectAWS();
    }
  }
  client.loop();
  // Check for button press to reset configuration
  if (digitalRead(buttonPin) == LOW) {
    if (millis() - lastButtonPress > 3000) { // Debounce button press
      handleResetConfig();
      lastButtonPress = millis();
    }
  }

  unsigned long currentMillis = millis();

  // HC12 input check
  if (Serial2.available() > 0) {
    String message = Serial2.readStringUntil('\n');
    Serial.print("HC12 Received: ");
    StaticJsonDocument<200> receivedData;
    DeserializationError err = deserializeJson(receivedData, message);
    if (!err) {
      serializeJson(receivedData, Serial);
//      publishMQTT(PUB_asd, receivedData);
      Serial.println();
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
      rbSerial.println(message + "\n");
    }
    
  }

  if (rbSerial.available() > 0) {
    String message = rbSerial.readStringUntil('\n');
    //    Serial.print("Received From Relay Board: ");
    Serial.println(message);

    // Parse and process the received JSON data
    StaticJsonDocument<200> relayData;
    DeserializationError err = deserializeJson(relayData, message);
    if (!err) {
      if (relayData.containsKey("toggle") || relayData.containsKey("regulate")) {
        publishMQTT(PUB_status, relayData);
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

  if (mpu.getMotionInterruptStatus() && vibFound) {
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


  // Millis for 5 seconds interval
  if (currentMillis - previousMillis >= mdFreq * 60000) { //60000
    previousMillis = currentMillis;

    // Get sensor data
    lux = bh1750Found ? luxLevel() : 0;
    wl = lidarFound ? waterLevel() : 0;
    now = rtcFound ? rtc.now() : DateTime(timeClient.getEpochTime());
    sound_level = dbFound ? reg_db(PCBARTISTS_DBM, I2C_REG_DECIBEL) : 0;
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
    doc["moistureLevel"] = avgMoisture;
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
    Serial.println(deviceId);
    publishMQTT(PUB_md, doc);
  }
  if (irrActive) {
    if (millis() - irrMillis >= irrDelay * 1000) {
      // Print {"toggle": {"irrPin": 0}}
      StaticJsonDocument<20> toggleDoc;
      toggleDoc["toggle"][String(irrPin)] = 0;
      
      String toggleOffJson;
      serializeJson(toggleDoc, toggleOffJson);
      rbSerial.println(toggleOffJson + "\n");

      // Reset the state
      irrActive = false;
    }
  }
  // Handle the logic for npkPin (pin 9)
  if (npkActive) {
    // Check if the delay period has passed
    if (millis() - npkMillis >= npkDelay * 1000) {
      // Print {"toggle": {"9": 0}}
      StaticJsonDocument<20> toggleDoc;
      toggleDoc["toggle"]["9"] = 0;
      String toggleOffJson;
      serializeJson(toggleDoc, toggleOffJson);
      rbSerial.println(toggleOffJson + "\n");
      // Reset the state
      npkActive = false;
    }
  }
}
void checkPowerStateChange(int &currentState, int &lastState, const char* alertType) {
    if (currentState != lastState) {
        if (currentState == LOW) {
            alertTrigger(alertType, 1);
        } else {
            alertTrigger(alertType, 0);
        }
        lastState = currentState; // Update the last known state
    }
}

void connectAWS() {
  Serial.println("Connecting to AWS IoT");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(deviceId);
    while (!client.connect(THINGNAME.c_str())) {
      Serial.println(client.state());
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
      
      awsConnected = true;  // Set the flag to true
    } else {
      Serial.println("AWS IoT Timeout!");
      awsConnected = false;  // Ensure flag is false if connection fails
    }
  } else {
    Serial.println("WiFi is not connected");
    awsConnected = false;
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
    publishMQTT(PUB_trig, alert);
}

float waterLevel() {
  // Test distance sensor
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    int c = measure.RangeMilliMeter / 10;
    return (c < 4) ? 100 : (c > 24) ? 0 : map(c, 4, 24, 0, 100);
  } else {
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
  StaticJsonDocument<400> sensorStatus;
  sensorStatus["lux"] = bh1750Found ? 1 : 0;
  sensorStatus["waterLevel"] = lidarFound ? 1 : 0;
  sensorStatus["db"] = dbFound ? 1 : 0;
  sensorStatus["rtc"] = rtcFound ? 1 : 0;
  sensorStatus["vib"] = vibFound ? 1 : 0;
  sensorStatus["moisture"] = 1;
  sensorStatus["relayBoard"] = 1;
  sensorStatus["dht"] = dhtFound ? 1 : 0;
  sensorStatus["pms"] = pmsFound ? 1 : 0;
  //  sensorStatus["vl53l0x"] = VL53L0XFound ? 1 : 0;

  // Serialize the JSON object and send it over the serial port
  serializeJsonPretty(sensorStatus, Serial);
  Serial.println();
  publishMQTT(PUB_senST, sensorStatus);
}

void publishMQTT(const String& topic, const JsonDocument& doc) {
  if (deviceId != "") {
    serializeJson(doc, mqttBuffer, MQTT_BUFFER_SIZE);
    Serial.print("Publishing to topic: ");
    Serial.println(topic);
    bool success = client.publish(topic.c_str(), mqttBuffer);
    if (success) {
      Serial.println("Publish successful");
    } else {
      Serial.print("Failed to publish, MQTT state: ");
      Serial.println(client.state());
    }
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
