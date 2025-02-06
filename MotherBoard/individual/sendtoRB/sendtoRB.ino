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

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 13, 12); // Initialize Serial3 for HC-12 communication
  rbSerial.begin(9600);    // Initialize Serial2 for relay board communication
  Wire.begin(1, 2);
}

void loop() {
  
  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n');
      rbSerial.println(message + "\n");
    }

  if (rbSerial.available() > 0) {
    String message = rbSerial.readStringUntil('\n');
    //    Serial.print("Received From Relay Board: ");
    Serial.println(message);
  }
}
