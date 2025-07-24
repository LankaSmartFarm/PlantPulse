#include <DS3231_RTC.h>

DS3231_RTC rtc;

volatile bool alarmTriggered = false;

void alarmISR() {
  alarmTriggered = true;
}

void setup() {
  Serial.begin(115200);
  rtc.begin();

  // Set current time & date (adjust this to your current time)
  rtc.setTime(8, 43, 0);
  rtc.setDate(2025, 7, 24, 5);  // Saturday (Sunday=1)

  // Set monthly alarm for 24th day at 08:20:00
  rtc.setAlarm1(DS3231_RTC::ALARM_MONTHLY, 8, 45, 10, 24);

  rtc.enableAlarmInterrupt(true);

  pinMode(2, INPUT_PULLUP);  // Assuming INT pin connected to Arduino pin 2
  attachInterrupt(digitalPinToInterrupt(2), alarmISR, FALLING);

  Serial.println("RTC Alarm example started");
}

void loop() {
  if (alarmTriggered) {
    alarmTriggered = false;

    uint16_t year;
    uint8_t month, day, weekday;
    rtc.getDate(year, month, day, weekday);

    Serial.print("Alarm triggered on ");
    Serial.print(year); Serial.print("-");
    Serial.print(month); Serial.print("-");
    Serial.print(day);
    Serial.print(" weekday: "); Serial.println(weekday);


    rtc.clearAlarm1Flag();

  }
}
