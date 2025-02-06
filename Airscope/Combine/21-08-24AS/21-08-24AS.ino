#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <DNSServer.h>
#include "secrets.h"  // Contains AWS MQTT credentials
#include <Adafruit_NeoPixel.h>
#include <ESPmDNS.h>

#define PIN 12
#define NUMPIXELS 1
const int buttonPin = 4; // GPIO pin connected to the push button
const unsigned long longPressTime = 3000; // 3 seconds in milliseconds
unsigned long buttonPressTime = 0;
bool buttonState = HIGH;
bool lastButtonState = HIGH;
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Configuration for WebServer and Wi-Fi management
WebServer server(80);
Preferences preferences;
DNSServer dnsServer;

const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 4, 1);
const char* hostName = "jbconfig";
const char* ssidAP = "JustBreathe-Config";
const char* passwordAP = "12345678";

// AWS IoT credentials and MQTT setup
WiFiClientSecure net;
PubSubClient client(net);
String THINGNAME;
String SUB_Config;
String SUB_Status;
String PUB_md;
String PUB_status;
String PUB_senST;
String PUB_trig;
String deviceId = "";
bool indoorSelected;
bool outdoorSelected;
bool awsConnected = false;  // Flag to track AWS connection status
// HTML Page Content
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
            <input type="button" value="Connect to Wi-Fi" class="connect-btn" onclick="connectToWiFi()">
            <div id="wifi-status" class="status">Wi-Fi Status: Not Connected</div>
        </form>
    </div>
    <div class="container" id="config-card">
        <h2>Device Configuration</h2>
        <form id="config-form">
            <label for="device-id">Device ID:</label>
            <input type="text" id="device-id" name="device-id">
            <label>Select Configuration:</label>
            <input type="checkbox" id="indoor" name="config-option" value="Indoor"> Indoor (RF)<br>
            <input type="checkbox" id="outdoor" name="config-option" value="Outdoor"> Outdoor (Cloud)<br>
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
                    } else {
                        wifiStatusDiv.textContent = "Wi-Fi Status: Not Connected";
                        setTimeout(checkWifiStatus, 5000);
                    }
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

void setup() {
  Serial.begin(115200);
  pixels.begin();
  pinMode(buttonPin, INPUT_PULLUP); // Set the button pin as input with internal pull-up resistor

  // Initialize NVS and load saved Wi-Fi credentials
  preferences.begin("wifi-config", false);
  String savedSSID = preferences.getString("ssid", "");
  String savedPassword = preferences.getString("password", "");
  preferences.end();

  if (savedSSID == "") {
    // Start Access Point if no saved Wi-Fi credentials
    WiFi.softAP(ssidAP, passwordAP);
    dnsServer.start(DNS_PORT, "*", apIP);

    Serial.println("Access Point started");
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());

    if (!MDNS.begin(hostName)) {
      Serial.println("Error starting mDNS");
    } else {
      Serial.printf("mDNS responder started at http://%s.local\n", hostName);
    }

    server.on("/", handleRoot);
    server.on("/save-config", HTTP_POST, handleWifiConfig);
    server.on("/save-device-config", HTTP_POST, handleSaveDeviceConfig);
    server.on("/reset-config", HTTP_POST, handleResetConfig);
    server.on("/wifi-networks", HTTP_GET, handleScanNetworks);
    server.on("/wifi-status", HTTP_GET, handleWifiStatus);
    server.begin();
    Serial.println("HTTP server started");

    setColor(255, 0, 0);  // Red LED for AP mode
  } else {
    // Attempt to connect to saved Wi-Fi credentials
    WiFi.mode(WIFI_STA);
    WiFi.begin(savedSSID.c_str(), savedPassword.c_str());
    Serial.println("Connecting to saved Wi-Fi...");
    int counter = 0;
    while (WiFi.status() != WL_CONNECTED && counter < 10) {
      delay(1000);
      Serial.print(".");
      counter++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nConnected to Wi-Fi");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      setColor(0, 0, 255);  // Blue LED for Wi-Fi connected
    } else {
      Serial.println("\nFailed to connect to saved Wi-Fi");
      setColor(255, 0, 0);  // Red LED for connection failure
    }
  }

  // Configure WiFiClientSecure for AWS IoT credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  client.setServer(AWS_IOT_ENDPOINT, 8883);

  // Load device configuration from NVS
  preferences.begin("device-config", false);
  deviceId = preferences.getString("deviceId", "");
  preferences.end();

  if (deviceId != "") {
    THINGNAME = deviceId;
    SUB_Config = "Devices/C2M/" + deviceId + "/config";
    SUB_Status = "Devices/C2M/" + deviceId + "/status";
    PUB_md = "Devices/M2C/" + deviceId + "/stationData";
    PUB_status = "Devices/M2C/" + deviceId + "/status";
    PUB_senST = "Devices/M2C/" + deviceId + "/sensorStatus";
    PUB_trig = "Devices/M2C/" + deviceId + "/triggers";

    // Don't connect to AWS IoT here, wait until after device ID is configured
  } else {
    Serial.println("No Device ID found");
  }
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();
  if(outdoorSelected){
  if (!awsConnected) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println(deviceId);
      connectAWS();
    } else {
      Serial.println("WiFi is not connected");
    }
  }
  client.loop();
  }
  // Check for button press to reset configuration
  buttonState = digitalRead(buttonPin);
  if (buttonState == LOW && lastButtonState == HIGH) {
    buttonPressTime = millis();
  } else if (buttonState == HIGH && lastButtonState == LOW) {
    if (millis() - buttonPressTime >= longPressTime) {
      Serial.println("Button long pressed for 3 seconds");
      handleResetConfig();
    }
  }
  lastButtonState = buttonState;
}

void handleRoot() {
  server.send_P(200, "text/html", index_html);
}

void handleWifiConfig() {
  if (server.method() == HTTP_POST) {
    String ssid = server.arg("ssid");
    String password = server.arg("password");

    preferences.begin("wifi-config", false);
    preferences.putString("ssid", ssid);
    preferences.putString("password", password);
    preferences.end();

    WiFi.begin(ssid.c_str(), password.c_str());

    int counter = 0;
    while (WiFi.status() != WL_CONNECTED && counter < 30) {
      delay(1000);
      Serial.print(".");
      counter++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      setColor(0, 0, 255);  // Blue for connected
      server.send(200, "application/json", "{\"message\":\"Connected to Wi-Fi\"}");
    } else {
      setColor(255, 0, 0);  // Red for failure
      server.send(200, "application/json", "{\"message\":\"Failed to connect to Wi-Fi\"}");
    }
  } else {
    server.send(405, "text/plain", "Method Not Allowed");
  }
}

void handleSaveDeviceConfig() {
  if (server.method() == HTTP_POST) {
    String deviceId = server.arg("device-id");
    indoorSelected = server.hasArg("config-option") && server.arg("config-option") == "Indoor";
    outdoorSelected = server.hasArg("config-option") && server.arg("config-option") == "Outdoor";

    preferences.begin("device-config", false);
    preferences.putString("deviceId", deviceId);
    preferences.putBool("indoor", indoorSelected);
    preferences.putBool("outdoor", outdoorSelected);
    preferences.end();

    THINGNAME = deviceId;
    SUB_Config = "Devices/C2M/" + deviceId + "/config";
    SUB_Status = "Devices/C2M/" + deviceId + "/status";
    PUB_md = "Devices/M2C/" + deviceId + "/stationData";
    PUB_status = "Devices/M2C/" + deviceId + "/status";
    PUB_senST = "Devices/M2C/" + deviceId + "/sensorStatus";
    PUB_trig = "Devices/M2C/" + deviceId + "/triggers";

    server.send(200, "application/json", "{\"message\":\"Device Configuration Saved.\"}");

    // Attempt to connect to AWS only after the device ID is configured and the outdoor option is selected
    if (outdoorSelected && WiFi.status() == WL_CONNECTED) {
      connectAWS();
    }
  } else {
    server.send(405, "text/plain", "Method Not Allowed");
  }
}

void handleResetConfig() {
  preferences.begin("wifi-config", false);
  preferences.clear();
  preferences.end();
  server.send(200, "application/json", "{\"message\":\"Configuration Reset. Please Restart ESP32.\"}");
  Serial.println("Configuration Reset. Please Restart ESP32.");
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

void connectAWS() {
  Serial.println("Connecting to AWS IoT...");
  if (WiFi.status() == WL_CONNECTED) {
    while (!client.connect(THINGNAME.c_str())) {
      Serial.println(client.state());
      delay(100);
    }
    if (client.connected()) {
      Serial.println("AWS IoT Connected!");
      client.subscribe(SUB_Config.c_str());
      client.subscribe(SUB_Status.c_str());
    } else {
      Serial.println("AWS IoT Connection Failed!");
    }
  } else {
    Serial.println("Wi-Fi not connected, cannot connect to AWS IoT");
  }
}

void setColorFromHex(const char* hexColor) {
  if (hexColor[0] == '#') {
    hexColor++;
  }
  long number = strtol(hexColor, NULL, 16);
  int redValue = (number >> 16) & 0xFF;
  int greenValue = (number >> 8) & 0xFF;
  int blueValue = number & 0xFF;
  setColor(redValue, greenValue, blueValue);
}

void setColor(int redValue, int greenValue, int blueValue) {
  pixels.setPixelColor(0, pixels.Color(redValue, greenValue, blueValue));
  pixels.show();
}
