#include <HardwareSerial.h>

HardwareSerial MySerial(1);

void setup() {
  Serial.begin(115200);
  MySerial.begin(9600, SERIAL_8N1, 17, 16);  // UART1 on GPIO 17 (TX) and GPIO 16 (RX)
}

void loop() {
  if (MySerial.available()) {
    String received = MySerial.readStringUntil('\n');
    Serial.print("Received from ESP32S3: ");
    Serial.println(received);
  }
}
