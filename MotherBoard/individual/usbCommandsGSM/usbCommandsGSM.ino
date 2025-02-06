#include <USB.h>

// Create a USBHost object
USBHost usb;

// USB serial device (GSM modem)
USBH_CDCACM gsm(&usb);

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for the serial monitor to open

  Serial.println("Starting USB Host");
  usb.begin();

  // Wait for the GSM modem to connect
  while (!gsm.connected()) {
    usb.Task();
  }

  Serial.println("GSM Modem connected");
  
  // Send AT commands
  sendAtCommand("AT+CGMI\r");  // Return information of manufacturer
  delay(1000);
  sendAtCommand("AT+CGMM\r");  // Return information of model
  delay(1000);
  sendAtCommand("AT+CGMR\r");  // Return information of revision
  delay(1000);
  sendAtCommand("AT+CGSN\r");  // Return information of serial number
  delay(1000);
}

void loop() {
  usb.Task();
  receiveAtResponse();
}

void sendAtCommand(const char* command) {
  if (gsm.connected()) {
    Serial.print("Sending command: ");
    Serial.println(command);
    gsm.write((const uint8_t*)command, strlen(command));
  }
}

void receiveAtResponse() {
  if (gsm.available()) {
    while (gsm.available()) {
      int c = gsm.read();
      Serial.write(c);
    }
    Serial.println();
  }
}
