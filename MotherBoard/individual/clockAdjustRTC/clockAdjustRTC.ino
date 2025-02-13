#include "RTCDS1307.h"
#include <Wire.h>
RTCDS1307 rtc(0x68);

uint8_t year, month, weekday, day, hour, minute, second;
bool period = 0;

String m[12] = {"Jan", "Feb", "Mar", "Apr", "May", "June", "July", "Aug", "Sep", "Oct", "Nov", "Dec"};
String w[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

void setup()
{
  Serial.begin(115200);
  Wire.begin(1,2);
  rtc.begin();
  rtc.setDate(19, 2, 28);
  rtc.setTime(23, 59, 50);
}

void loop()
{
  rtc.getDate(year, month, day, weekday);
  rtc.getTime(hour, minute, second, period);
  if (!(second % 3)) rtc.setMode(1 - rtc.getMode());
  rtc.getTime(hour, minute, second, period);

  Serial.print(w[weekday - 1]);
  Serial.print("  ");
  Serial.print(day, DEC);
  Serial.print("/");
  Serial.print(m[month - 1]);
  Serial.print("/");
  Serial.print(year + 2000, DEC);
  Serial.print("  ");
  Serial.print(hour, DEC);
  Serial.print(":");
  Serial.print(minute, DEC);
  Serial.print(":");
  Serial.print(second, DEC);
  Serial.print(rtc.getMode() ? (period ? " PM" : " AM") : "");
  Serial.println();
  delay(1000);
}
