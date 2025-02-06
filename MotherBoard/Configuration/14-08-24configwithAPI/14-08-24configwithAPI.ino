#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <Ticker.h>

const char* ssidAP = "ESP32-Config";
const char* passwordAP = "12345678";
WebServer server(80);
Preferences preferences;
DNSServer dnsServer;

const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 4, 1);
const char* hostName = "esp32config";

Ticker ledTicker;
bool ledState = false;
unsigned long lastButtonPress = 0;

const int buttonPin = 14;
const int redPin = 38;
const int greenPin = 39;
const int bluePin = 40;

const char index_html[] PROGMEM = R"rawliteral(
  // Place your HTML code here
  <!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32 Configuration</title>
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
    <div class="container disabled" id="config-card">
        <h2>Device Configuration</h2>
        <form id="config-form">
            <label for="cert1">Certificate File 1:</label>
            <textarea id="cert1" name="cert1" rows="2"></textarea>
            <label for="cert2">Certificate File 2:</label>
            <textarea id="cert2" name="cert2" rows="2"></textarea>
            <label for="cert3">Certificate File 3:</label>
            <textarea id="cert3" name="cert3" rows="2"></textarea>
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
    String ssid = server.arg("ssid");
    String password = server.arg("password");

    // Save Wi-Fi credentials to preferences
    preferences.begin("wifi-config", false);
    preferences.putString("ssid", ssid);
    preferences.putString("password", password);
    preferences.end();

    server.send(200, "application/json", "{\"message\":\"Wi-Fi Configuration Received. Connecting...\"}");

    // Attempt to connect to Wi-Fi
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
      digitalWrite(redPin, LOW);
      digitalWrite(bluePin, LOW);
      digitalWrite(greenPin, HIGH); // Connected with internet working
      ledTicker.detach(); // Stop blinking
    } else {
      Serial.println("\nFailed to connect to Wi-Fi");
      digitalWrite(redPin, HIGH);
      digitalWrite(bluePin, LOW);
      digitalWrite(greenPin, LOW); // Not connected to Wi-Fi
    }
  } else {
    server.send(405, "text/plain", "Method Not Allowed");
  }
}

void handleSaveDeviceConfig() {
  if (server.method() == HTTP_POST) {
    String cert1 = server.arg("cert1");
    String cert2 = server.arg("cert2");
    String cert3 = server.arg("cert3");
    String deviceId = server.arg("device-id");

    // Print received data to serial monitor
    Serial.println("Received Device Configuration:");
    Serial.println("Device ID: " + deviceId);
    Serial.println("Certificate 1: " + cert1);
    Serial.println("Certificate 2: " + cert2);
    Serial.println("Certificate 3: " + cert3);


    preferences.begin("device-config", false);
    preferences.putString("cert1", cert1);
    preferences.putString("cert2", cert2);
    preferences.putString("cert3", cert3);
    preferences.putString("deviceID", deviceId);
    preferences.end();

    server.send(200, "application/json", "{\"message\":\"Device Configuration Saved.\"}");
  } else {
    server.send(405, "text/plain", "Method Not Allowed");
  }
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

void setup() {
  Serial.begin(115200);

  // Initialize NVS
  preferences.begin("wifi-config", false);
  String savedSSID = preferences.getString("ssid", "");
  String savedPassword = preferences.getString("password", "");
  preferences.end();

  // Initialize button and LED pins
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

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
    WiFi.begin(savedSSID.c_str(), savedPassword.c_str());
    Serial.println("Connecting to saved Wi-Fi...");
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

      // Check if internet is available
      if (WiFi.status() == WL_CONNECTED) {
        digitalWrite(redPin, LOW);
        digitalWrite(bluePin, LOW);
        digitalWrite(greenPin, HIGH); // Connected with internet working
      } else {
        digitalWrite(redPin, LOW);
        digitalWrite(bluePin, HIGH);
        digitalWrite(greenPin, LOW); // Connected to Wi-Fi but no internet
      }
    } else {
      Serial.println("\nFailed to connect to saved Wi-Fi");
      digitalWrite(redPin, HIGH);
      digitalWrite(bluePin, LOW);
      digitalWrite(greenPin, LOW); // Not connected to Wi-Fi
    }
  }
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();

  // Check for button press to reset configuration
  if (digitalRead(buttonPin) == LOW) {
    if (millis() - lastButtonPress > 3000) { // Debounce button press
      handleResetConfig();
      lastButtonPress = millis();
    }
  }
}
