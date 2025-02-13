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
#include <Adafruit_NeoPixel.h>
//Sensor Libraries
#include <BH1750.h>
#include "DHT.h"
#include "PMS.h"

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
const unsigned long longPressTime = 3000; // 3 seconds in milliseconds
unsigned long buttonPressTime = 0;
bool buttonState = HIGH;
bool lastButtonState = HIGH;
//GPIO's
const int buttonPin = 4; // GPIO pin connected to the push button
#define DHTPIN 27
const int PIN = 12;
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


//Future nvs config variables
//String deviceId;
String THINGNAME;
String SUB_Config;
String PUB_senST;
String PUB_asd;

WiFiClientSecure net;
PubSubClient client(net);
//Preferences preferences;
// Increase MQTT buffer size
const int MQTT_BUFFER_SIZE = 1024;
char mqttBuffer[MQTT_BUFFER_SIZE];

// Function prototypes
void messageHandler(char* topic, byte* payload, unsigned int length);
void connectAWS();

//Object Initialization
BH1750 lightMeter;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800, 60000); // IST offset 19800 seconds (5.5 hours)
RTC_DS3231 rtc;
#define PCBARTISTS_DBM 0x48
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);

#define sw_rs 32
#define sw_tx 33
EspSoftwareSerial::UART SoftSerial;

#define pms_serial SoftSerial
PMS pms(pms_serial);

//PM25AQI Initialize
void PM25AQI_init() {
  pms_serial.begin(9600, EspSoftwareSerial::SWSERIAL_8N1, sw_rs, sw_tx, false, 256);
  // high speed half duplex, turn off interrupts during tx
  pms_serial.enableIntTx(false);
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
//const unsigned long interval = 60000; // Interval in milliseconds (5 seconds)
unsigned long previousMillis = 0;    // Store the last time the message was printed

unsigned long previousMillisasd = 0;    // Store the last time the message was printed



bool bh1750Found;
bool dbFound;
bool rtcFound;
bool o2Found;
bool co2Found;
bool micsFound;
bool o3Found;
bool tvocFound;
bool pmsFound;
bool dhtFound;
bool owtFound;

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

float lux;
byte sound_level;
int asdFreq = 60;
unsigned long currentMillisasd = millis();
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
        .aws-connected {
    color: green;
}

.aws-disconnected {
    color: red;
}
.connected {
    color: green;
}

.disconnected {
    color: red;
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
        <div id="wifi-status" class="status disconnected">Wi-Fi Status: Not Connected</div>
        <input type="button" value="Connect to Wi-Fi" class="connect-btn" onclick="connectToWiFi()">
    </form>
</div>

<div class="container" id="aws-card">
    <div id="aws-status" class="status disconnected">AWS Status: Not Connected</div>
</div>

<div class="container disabled" id="config-card">
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
                wifiStatusDiv.classList.add('connected');
                wifiStatusDiv.classList.remove('disconnected');
                enableConfigCard();  // Enable the configuration card
            } else {
                wifiStatusDiv.textContent = "Wi-Fi Status: Not Connected";
                wifiStatusDiv.classList.add('disconnected');
                wifiStatusDiv.classList.remove('connected');
                setTimeout(checkWifiStatus, 5000);  // Retry after 5 seconds
            }
        })
        .catch(error => {
            console.error('Error checking Wi-Fi status:', error);
            setTimeout(checkWifiStatus, 5000);  // Retry after 5 seconds
        });
}


function enableConfigCard() {
    const configCard = document.getElementById('config-card');
    configCard.classList.remove('disabled');
}

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
    event.preventDefault();  // Prevent the default form submission
    const formData = new FormData(this);

    fetch('/save-device-config', {
        method: 'POST',
        body: formData
    })
    .then(response => response.json())
    .then(data => {
        alert(data.message);
        if (data.message.includes('connected to AWS')) {
            alert('Device successfully connected to AWS IoT. Closing configuration page.');
            window.close();  // Close the page if AWS is connected
        } else {
            alert('Configuration saved, but AWS connection failed.');
        }
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

function checkAwsStatus() {
    fetch('/aws-status')
        .then(response => response.json())
        .then(data => {
            const awsStatusDiv = document.getElementById('aws-status');
            if (data.connected) {
                awsStatusDiv.textContent = "AWS Status: Connected";
                awsStatusDiv.classList.add('connected');
                awsStatusDiv.classList.remove('disconnected');
            } else {
                awsStatusDiv.textContent = "AWS Status: Not Connected";
                awsStatusDiv.classList.add('disconnected');
                awsStatusDiv.classList.remove('connected');
                setTimeout(checkAwsStatus, 5000);  // Retry after 5 seconds
            }
        })
        .catch(error => {
            console.error('Error checking AWS status:', error);
            setTimeout(checkAwsStatus, 5000);  // Retry after 5 seconds
        });
}

document.addEventListener('DOMContentLoaded', function() {
    checkWifiStatus();  // Start checking Wi-Fi status when the page loads
    checkAwsStatus();   // Start checking AWS status when the page loads
});



    
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
void handleAwsStatus() {
    DynamicJsonDocument json(256);
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

    THINGNAME = deviceId;

    SUB_Config = "Devices/C2M/" + deviceId + "/config";
    PUB_senST = "Devices/M2C/" + deviceId + "/sensorStatus";
    PUB_asd = "Devices/M2C/" + deviceId + "/airScopeData";

    // Try to connect to AWS IoT only after the device ID is saved
    connectAWS(); // Attempt to connect to AWS IoT

    if (awsConnected) {
      server.send(200, "application/json", "{\"message\":\"Device Configuration Saved and connected to AWS.\"}");
    } else {
      server.send(200, "application/json", "{\"message\":\"Device Configuration Saved but failed to connect to AWS.\"}");
    }
  } else {
    server.send(405, "text/plain", "Method Not Allowed");
  }
}


void handleResetConfig() {
  preferences.begin("wifi-config", false);
  preferences.clear();
  preferences.end();

  preferences.begin("device-config", false);
  preferences.clear();  // Clear device ID and other device-specific configs
  preferences.end();

  // Disconnect MQTT client if connected
    if (client.connected()) {
        client.disconnect();
    }
    
  deviceId = "";
  THINGNAME = "";
    SUB_Config = "";
    PUB_senST = "";
    PUB_asd = "";
    
  server.send(200, "application/json", "{\"message\":\"Configuration Reset. Please Restart ESP32.\"}");
  Serial.println("Configuration Reset. Please Restart ESP32.");
   // Disconnect from Wi-Fi and clear credentials
  WiFi.disconnect(true, true);  // Disconnect and erase credentials
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
        json["ip"] = WiFi.localIP().toString();
    }
    String response;
    serializeJson(json, response);
    server.send(200, "application/json", response);
}
//

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // Initialize Serial2 for HC-12 communication
  Wire.begin();
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
    server.on("/aws-status", HTTP_GET, handleAwsStatus);
    server.begin();
    Serial.println("HTTP server started");

    // Start blinking red LED
    ledTicker.attach(0.5, []() {
    ledState = !ledState;
    if (ledState) {
        setColor(255, 0, 0);  // Red color
    } else {
        setColor(0, 0, 0);    // Turn off the LED
    }
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
//  Serial.println("Wifi COnnected");
  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  client.setServer(AWS_IOT_ENDPOINT, 8883);
  client.setCallback(messageHandler);
  
  preferences.begin("device-config", false);
  deviceId = preferences.getString("deviceId", "");
  preferences.end();
  THINGNAME = deviceId;
  SUB_Config = "Devices/C2M/" + deviceId + "/config";
  PUB_senST = "Devices/M2C/" + deviceId + "/sensorStatus";
  PUB_asd = "Devices/M2C/" + deviceId + "/airScopeData";
  
  if(deviceId != ""){
    if(WiFi.status() == WL_CONNECTED){
      Serial.println(deviceId);
      connectAWS();
    }
    else{
      Serial.println("Wifi is not Connected");
    }
  }
  else{
    Serial.println("Station Not alloted");
  }
  
  
  pinMode(buttonPin, INPUT_PULLUP); // Set the button pin as input with internal pull-up resistor

  Serial.println("Started");

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
  
  // Load configuration from NVS
  loadConfiguration();
//  alertTrigger("restart",1);
}

void loop() {
  currentMillisasd = millis();
//  Serial.println("Loop started");
  dnsServer.processNextRequest();
  server.handleClient();

  // Check Wi-Fi status and re-enable AP if disconnected
  if (WiFi.status() != WL_CONNECTED && !WiFi.isConnected()) {
//    Serial.println("Wi-Fi disconnected, restarting AP mode...");
    WiFi.softAP(ssidAP, passwordAP);
    dnsServer.start(DNS_PORT, "*", apIP);
//    Serial.print("AP IP Address: ");
//    Serial.println(WiFi.softAPIP());
  }
  // Only attempt to connect to AWS if not already connected
  if (!client.connected()) {
    if (WiFi.status() == WL_CONNECTED) {
      if(deviceId != "" && THINGNAME != ""){
      
      Serial.println(deviceId);
      connectAWS();
      }
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

  
  StaticJsonDocument<1024> doc;

  
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

  
  // Millis for 5 seconds interval
    if (currentMillisasd - previousMillisasd >= asdFreq * 1000) { //60000
  previousMillis = currentMillisasd;
    // Get sensor data
    if(bh1750Found){
      lux = luxLevel();
    }else{
      lux = 0;
    }
     if (rtcFound) {
            now = rtc.now();
        } else {
            timeClient.update();
            unsigned long epochTime = timeClient.getEpochTime();
            now = DateTime(epochTime);
        }
    if(dbFound){
    sound_level = reg_db(PCBARTISTS_DBM, I2C_REG_DECIBEL);
    }
    else{
    sound_level = 0;
    }
    h = dht.readHumidity();
    t = dht.readTemperature();
    update_pms_data();
    if (isnan(h) || isnan(t)) {
      h = 0.0;
      t = 0.0;
    }
    

    // Create a JSON document
    StaticJsonDocument<1024> doc;
    doc["pm1"] = pms25_data[0];
    doc["pm2_5"] = pms25_data[1];
    doc["pm10"] = pms25_data[2];
    doc["temperature"] = t;
    doc["humidity"] = h;
    doc["lux"] = lux;
    doc["db"] = sound_level;

    String dateTime = String(now.year()) + "-" + String(now.month()) + "-" + String(now.day()) + " " + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
    doc["dateTime"] = dateTime;

    serializeJsonPretty(doc, Serial);
    serializeJson(doc, Serial2);
    Serial.println(deviceId);
    if(deviceId != ""){
      serializeJson(doc, mqttBuffer, MQTT_BUFFER_SIZE);
      Serial.print("Publishing station data: ");
      Serial.println(mqttBuffer);
      bool success = client.publish(PUB_asd.c_str(), mqttBuffer);
      if (success) {
        Serial.println("Station data published successfully");
      } else {
        Serial.print("Failed to publish station data, MQTT state: ");
        Serial.println(client.state());
        Serial.print("Payload size: ");
        Serial.println(strlen(mqttBuffer));
      }
    }


    //    setColor(random(0, 255), random(0, 255), random(0, 255));
    //    rbSerial.println("Hello From Mother Board");
  }
}


void connectAWS() {
//    Serial.println("Connecting to AWS IoT");
    if (WiFi.status() == WL_CONNECTED) {
      if(THINGNAME != "" && deviceId != ""){
        while (!client.connect(THINGNAME.c_str())) {
            Serial.println(client.state());
            Serial.print(".");
            delay(100);
        }
        if (client.connected()) {
            Serial.println("AWS IoT Connected!");
            client.subscribe(SUB_Config.c_str());
//            client.subscribe(SUB_Status.c_str());
            awsConnected = true;  // Set awsConnected to true only after successful connection
            setColorFromHex("#00FF00");
        } else {
            Serial.println("AWS IoT Timeout!");
            awsConnected = false;
        }
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
//  Serial.println();

  // Perform logic based on the topic
  if (strcmp(topic, SUB_Config.c_str()) == 0) {
    Serial.println("Processing configuration data...");
    saveConfiguration(doc.as<JsonObject>());  // Save the configuration to NVS
  } 
}

float luxLevel() {
    return lightMeter.readLightLevel();
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
  pixels.setPixelColor(0, pixels.Color(redValue, greenValue, blueValue));
  pixels.show();
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
  dbFound = isAddressInArray("0x48");
  rtcFound = isAddressInArray("0x68");
  o2Found = isAddressInArray("0x68");
  co2Found = isAddressInArray("0x68");
  micsFound = isAddressInArray("0x68");
  o3Found = isAddressInArray("0x68");
  tvocFound = isAddressInArray("0x68");
  
//  pmsFound = isAddressInArray("0x68");
//  dhtFound = isAddressInArray("0x68");
//  owtFound = isAddressInArray("0x68");

  //  VL53L0XFound = isAddressInArray("0x29");
  StaticJsonDocument<400> sensorStatus;
  sensorStatus["asdlux"] = bh1750Found ? 1 : 0;
  sensorStatus["asddb"] = dbFound ? 1 : 0;
  sensorStatus["asdrtc"] = rtcFound ? 1 : 0;
  sensorStatus["asdo2"] = o2Found ? 1 : 0;
  sensorStatus["asdco2"] = co2Found ? 1 : 0;
  sensorStatus["asddht"] = dhtFound ? 1 : 0;
  sensorStatus["asdowt"] = owtFound ? 1 : 0;
  sensorStatus["asdmics"] = micsFound ? 1 : 0;
  sensorStatus["asdo3"] = o3Found ? 1 : 0;
  sensorStatus["asdtvoc"] = tvocFound ? 1 : 0;
  sensorStatus["asdpms"] = pmsFound ? 1 : 0;


  // Serialize the JSON object and send it over the serial port
  serializeJsonPretty(sensorStatus, Serial);
//  Serial.println();
  if(deviceId != ""){
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
}

void saveConfiguration(const JsonObject& config) {
  Preferences preferences;
  if (!preferences.begin("config", false)) {  // Open in read-write mode
    Serial.println("Failed to open NVS for writing");
    return;
  }

  preferences.putInt("asdFreq", config["asdFreq"]);
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

  asdFreq = preferences.getInt("asdFreq", 60) * 1000;
  float version = preferences.getFloat("version", 0.0);
  int update = preferences.getInt("update", 0);

  // Close the preferences as soon as you're done
  preferences.end();

  Serial.println("Loaded configuration from NVS:");
  Serial.println("asdFreq: " + String(asdFreq));
  Serial.println("version: " + String(version));
  Serial.println("update: " + String(update));
}
