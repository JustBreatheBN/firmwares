#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>

const char* ssidAP = "ESP32-Config";
const char* passwordAP = "12345678";
WebServer server(80);
Preferences preferences;
DNSServer dnsServer;

const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 4, 1);
const char* hostName = "esp32config";

const char index_html[] PROGMEM = R"rawliteral(
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
        }
        .container {
            background-color: #fff;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 0 10px rgba(0, 0, 0, 0.1);
            max-width: 400px;
            width: calc(100% - 40px);
            box-sizing: border-box;
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
        input[type="submit"] {
            background-color: #007bff;
            color: white;
            padding: 10px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            width: 100%;
            font-size: 16px;
            box-sizing: border-box;
        }
        input[type="submit"]:hover {
            background-color: #0056b3;
        }
    </style>
</head>
<body>
    <div class="container">
        <h2>ESP32 Configuration</h2>
        <form id="wifi-form">
            <label for="ssid">Select Wi-Fi Network:</label>
            <select id="ssid" name="ssid"></select>
            <label for="password">Wi-Fi Password:</label>
            <input type="password" id="password" name="password">
            <label for="cert1">Certificate File 1:</label>
            <textarea id="cert1" name="cert1" rows="2"></textarea>
            <label for="cert2">Certificate File 2:</label>
            <textarea id="cert2" name="cert2" rows="2"></textarea>
            <label for="device-id">Device ID:</label>
            <input type="text" id="device-id" name="device-id">
            <input type="submit" value="Save Configuration">
        </form>
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

            document.getElementById('wifi-form').addEventListener('submit', function(event) {
                event.preventDefault();
                const formData = new FormData(this);
                
                fetch('/save-config', {
                    method: 'POST',
                    body: formData
                })
                .then(response => response.json())
                .then(data => {
                    alert(data.message);
                })
                .catch(error => {
                    console.error('Error:', error);
                    alert('Failed to save configuration.');
                });
            });
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
    String ssid = server.arg("ssid");
    String password = server.arg("password");
    String deviceId = server.arg("device-id");
    String cert1 = server.arg("cert1");
    String cert2 = server.arg("cert2");

    // Print received data to serial monitor
    Serial.println("Received Configuration:");
    Serial.println("SSID: " + ssid);
    Serial.println("Password: " + password);
    Serial.println("Device ID: " + deviceId);
    Serial.println("Certificate 1: " + cert1);
    Serial.println("Certificate 2: " + cert2);

    preferences.begin("wifi-config", false);
    preferences.putString("ssid", ssid);
    preferences.putString("password", password);
    preferences.putString("deviceID", deviceId);
    preferences.putString("cert1", cert1);
    preferences.putString("cert2", cert2);
    preferences.end();

    server.send(200, "application/json", "{\"message\":\"Configuration Received. Check Serial Monitor.\"}");

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
    } else {
      Serial.println("\nFailed to connect to Wi-Fi");
    }
  } else {
    server.send(405, "text/plain", "Method Not Allowed");
  }
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
  
  // Temporarily clear the saved Wi-Fi credentials
  preferences.clear();
  preferences.end();

  // Start access point mode
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
  server.on("/wifi-networks", HTTP_GET, handleScanNetworks);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();
}
