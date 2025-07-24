#ifndef DS3231_RTC_H
#define DS3231_RTC_H

#include <Arduino.h>
#include <Wire.h>

class DS3231_RTC {
public:
  DS3231_RTC();

  void begin();

  enum AlarmMode {
    ALARM_OFF,
    ALARM_HOURLY,
    ALARM_WEEKLY,
    ALARM_MONTHLY,
    ALARM_YEARLY
  };

  void setTime(uint8_t hour, uint8_t minute, uint8_t second);
  void getTime(uint8_t &hour, uint8_t &minute, uint8_t &second);

  void setDate(uint16_t year, uint8_t month, uint8_t day, uint8_t weekday);
  void getDate(uint16_t &year, uint8_t &month, uint8_t &day, uint8_t &weekday);

  void setAlarm1(AlarmMode mode, uint8_t hour=0, uint8_t minute=0, uint8_t second=0, uint8_t dayOrWeekday=1);

  void clearAlarm1Flag();

  void enableAlarmInterrupt(bool enable);

  bool isAlarm1Triggered();

private:
  uint8_t decToBcd(uint8_t val);
  uint8_t bcdToDec(uint8_t val);
};

#endif
