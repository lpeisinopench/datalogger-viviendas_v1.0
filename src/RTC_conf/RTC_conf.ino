#include <Wire.h>
#include <RTClib.h>
 
RTC_DS3231 rtc;
 
void setup () {
 Serial.begin(9600);
 rtc.begin();
 // Fijar a fecha y hora específica. En el ejemplo, 21 de Enero de 2016 a las 03:00:00
 // rtc.adjust(DateTime(2016, 1, 21, 3, 0, 0));
//rtc.adjust(DateTime(2021,12,20,13,51,00));
// Fijar a fecha y hora específica. En el ejemplo, 06 de Febrero de 2022 a las 21:31:00
//rtc.adjust(DateTime(2022,02,06,21,31,00));
}
 
void loop () {
 DateTime now = rtc.now();
 Serial.print(now.day());
 Serial.print('/');
 Serial.print(now.month());
 Serial.print('/');
 Serial.print(now.year());
 Serial.print(" ");
 Serial.print(now.hour());
 Serial.print(':');
 Serial.print(now.minute());
 Serial.print(':');
 Serial.print(now.second());
 Serial.println();
 delay(2000);
  
}

