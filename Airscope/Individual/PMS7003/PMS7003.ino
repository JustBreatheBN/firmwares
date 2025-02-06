
#include "PMS.h"
#include <SoftwareSerial.h>

#define sw_rs 32
#define sw_tx 33
EspSoftwareSerial::UART SoftSerial;

#define pms_serial SoftSerial
PMS pms(pms_serial);

//------------------------PM25AQI_initialize-----------------------------------------------------
void PM25AQI_init() {
  pms_serial.begin(9600, EspSoftwareSerial::SWSERIAL_8N1, sw_rs, sw_tx, false, 256);
  // high speed half duplex, turn off interrupts during tx
  pms_serial.enableIntTx(false);
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
PMS::DATA pms_data;
int pms25_data[3];
void update_pms_data() {
  pms_serial.listen();
  pms.readUntil(pms_data);
  pms25_data[0] = pms_data.PM_AE_UG_1_0;
  pms25_data[1] = pms_data.PM_AE_UG_2_5;
  pms25_data[2] = pms_data.PM_AE_UG_10_0;
}
//-----------------------------------------------------------------------------

void setup() {
  
  Serial.begin(115200);
  PM25AQI_init();
  Serial.println("ok pm25");  //1
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void loop() {
  data_loop();
}
void data_loop() {
  
  update_pms_data();
  
  String Data = "{";
  //  Data += "\"lat\":" + String(12.9858922) + ",";
  //  Data += "\"longt\":" + String(77.7250001) + ",";
  
  Data += "\"pm1\":" + String(pms25_data[0]) + ",";
  Data += "\"pm25\":" + String(pms25_data[1]) + ",";
  Data += "\"pm10\":" + String(pms25_data[2]) + "";
  
  Data += "}";
  Serial.println(Data);
  delay(2000);

}
