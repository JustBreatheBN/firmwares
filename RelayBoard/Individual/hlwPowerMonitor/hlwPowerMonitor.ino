#include "HLW8032.h"

#define SYNC_TIME 2000

static unsigned long start_time, end_time;
HLW8032 HL;

void setup()
{
  Serial.begin(115200); // Start serial communication at 115200 baud rate
  Serial2.begin(4800, SERIAL_8N1, 23, 22); // Initialize Serial2 with baud rate 4800, RX=23, TX=22 (adjust if needed)
  HL.begin(Serial2, -1); // Initialize HLW8032 with Serial2 and IO pin 4 (change if needed)
  start_time = millis();
}

void loop()
{
  end_time = millis();
  HL.SerialReadLoop();

  if (end_time - start_time > SYNC_TIME)
  {
    if (HL.SerialRead == 1)
    {


      Serial.println("-----------------------------------");
      Serial.print("Voltage[V]: ");
      Serial.println(HL.GetVol() * 0.001, 2);

      Serial.print("Voltage Parameter: ");
      Serial.println(HL.GetVolAnalog(), 2);

      Serial.print("Current[A]: ");
      Serial.println(HL.GetCurrent(), 2);

      Serial.print("Current Parameter: ");
      Serial.println(HL.GetCurrentAnalog(), 2);

      Serial.print("Power[W]: ");
      Serial.println(HL.GetActivePower() * 0.001, 2);

      Serial.print("Inspecting Power: ");
      Serial.println(HL.GetInspectingPower(), 2);

      Serial.print("Power Factor: ");
      Serial.println(HL.GetPowerFactor(), 2);

      Serial.print("PF Register: ");
      Serial.println(HL.GetPF(), 2);

      Serial.print("PF Total: ");
      Serial.println(HL.GetPFAll(), 2);

      Serial.print("Energy[KWh]: ");
      Serial.println(HL.GetKWh());
    }
    start_time = end_time;
  }
}
