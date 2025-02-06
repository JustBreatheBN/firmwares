#include <WiFi.h>
#include <HTTPUpdate.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h> // Required for HTTPS connections

// ======= CONFIGURATION =======
const char* ssid     = "Just Breathe";
const char* password = "9845055994";

// ======= TIMING VARIABLES =======
unsigned long previousMillis = 0;
const unsigned long printInterval = 5000; // 5 seconds

// ======= SETUP =======
void setup() {
  Serial.begin(115200);
  delay(1000); // Allow time for the Serial Monitor to open
  Serial.println("Starting OTA Update Test via JSON command...");

  // Connect to Wi-Fi.
  Serial.printf("Connecting to Wi-Fi: %s\n", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Instructions for triggering the OTA update.
  Serial.println("Send a JSON command (followed by newline) like:");
  Serial.println("{\"update\":1, \"url\":\"https://raw.githubusercontent.com/JustBreatheBN/firmwares/main/MotherBoard/firmware.bin\"}");
}

// ======= LOOP =======
void loop() {
  unsigned long currentMillis = millis();

  // Every 5 seconds, print a waiting message.
  if (currentMillis - previousMillis >= printInterval) {
    previousMillis = currentMillis;
    Serial.println("First Update from github");
  }

  // Check for incoming JSON commands via Serial.
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove whitespace/newlines
    if (input.length() > 0) {
      Serial.print("Received JSON: ");
      Serial.println(input);

      // Parse the JSON command.
      StaticJsonDocument<256> doc;
      DeserializationError error = deserializeJson(doc, input);
      if (error) {
        Serial.print("JSON parse error: ");
        Serial.println(error.c_str());
        return;
      }

      // Check if the command contains "update": 1.
      if (doc.containsKey("update") && doc["update"] == 1) {
        // Check for the URL in the JSON.
        if (doc.containsKey("url")) {
          const char* url = doc["url"];
          Serial.print("Starting OTA update with URL: ");
          Serial.println(url);

          // Determine whether to use HTTPS or HTTP.
          String urlStr(url);
          if (urlStr.startsWith("https")) {
            WiFiClientSecure client;
            client.setInsecure(); // For testing only. In production, use proper certificate validation.
            Serial.println("Using WiFiClientSecure for HTTPS update.");
            t_httpUpdate_return ret = httpUpdate.update(client, url);
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
            WiFiClient client;
            Serial.println("Using WiFiClient for HTTP update.");
            t_httpUpdate_return ret = httpUpdate.update(client, url);
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
}
