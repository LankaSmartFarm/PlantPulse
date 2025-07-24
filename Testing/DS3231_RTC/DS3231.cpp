#include "DS3231_RTC.h"

#define DS3231_ADDRESS 0x68
#define REG_SECONDS 0x00
#define REG_MINUTES 0x01
#define REG_HOURS 0x02
#define REG_DAY 0x03
#define REG_DATE 0x04
#define REG_MONTH 0x05
#define REG_YEAR 0x06
#define REG_ALARM1_SECONDS 0x07
#define REG_ALARM1_MINUTES 0x08
#define REG_ALARM1_HOURS 0x09
#define REG_ALARM1_DAY_DATE 0x0A
#define REG_CONTROL 0x0E
#define REG_STATUS 0x0F

DS3231_RTC::DS3231_RTC() {}

void DS3231_RTC::begin() {
  Wire.begin();

  // Enable interrupt control bits: INTCN = 1 to enable interrupts on INT pin
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(REG_CONTROL);
  Wire.endTransmission();

  Wire.requestFrom(DS3231_ADDRESS, (uint8_t)1);
  uint8_t control = Wire.read();

  control |= 0x04; // Set INTCN = 1
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(REG_CONTROL);
  Wire.write(control);
  Wire.endTransmission();

  clearAlarm1Flag();
}

uint8_t DS3231_RTC::decToBcd(uint8_t val) {
  return (val / 10 * 16) + (val % 10);
}

uint8_t DS3231_RTC::bcdToDec(uint8_t val) {
  return (val / 16 * 10) + (val % 16);
}

void DS3231_RTC::setTime(uint8_t hour, uint8_t minute, uint8_t second) {
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(REG_SECONDS);
  Wire.write(decToBcd(second));
  Wire.write(decToBcd(minute));
  Wire.write(decToBcd(hour));
  Wire.endTransmission();
}

void DS3231_RTC::getTime(uint8_t &hour, uint8_t &minute, uint8_t &second) {
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(REG_SECONDS);
  Wire.endTransmission();

  Wire.requestFrom(DS3231_ADDRESS, (uint8_t)3);
  second = bcdToDec(Wire.read());
  minute = bcdToDec(Wire.read());
  hour = bcdToDec(Wire.read());
}

void DS3231_RTC::setDate(uint16_t year, uint8_t month, uint8_t day, uint8_t weekday) {
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(REG_DAY);
  Wire.write(weekday);
  Wire.write(decToBcd(day));
  Wire.write(decToBcd(month));
  Wire.write(decToBcd(year - 2000));
  Wire.endTransmission();
}

void DS3231_RTC::getDate(uint16_t &year, uint8_t &month, uint8_t &day, uint8_t &weekday) {
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(REG_DAY);
  Wire.endTransmission();

  Wire.requestFrom(DS3231_ADDRESS, (uint8_t)4);
  weekday = Wire.read();
  day = bcdToDec(Wire.read());
  month = bcdToDec(Wire.read());
  year = bcdToDec(Wire.read()) + 2000;
}

void DS3231_RTC::setAlarm1(AlarmMode mode, uint8_t hour, uint8_t minute, uint8_t second, uint8_t dayOrWeekday) {
  uint8_t secReg = decToBcd(second) & 0x7F;
  uint8_t minReg = decToBcd(minute) & 0x7F;
  uint8_t hourReg = decToBcd(hour) & 0x7F;
  uint8_t dayDateReg = 0;

  uint8_t mask1 = 0x80;
  uint8_t mask2 = 0x80;
  uint8_t mask3 = 0x80;
  uint8_t mask4 = 0x80;

  switch(mode) {
    case ALARM_OFF:
      break;

    case ALARM_HOURLY:
      mask2 = 0x80;
      mask3 = 0x80;
      mask4 = 0x80;
      secReg = decToBcd(second);
      break;

    case ALARM_WEEKLY:
      mask1 = 0;
      mask2 = 0;
      mask3 = 0;
      mask4 = 0;
      dayDateReg = decToBcd(dayOrWeekday) | 0x40;
      secReg = decToBcd(second);
      minReg = decToBcd(minute);
      hourReg = decToBcd(hour);
      break;

    case ALARM_MONTHLY:
      mask1 = 0;
      mask2 = 0;
      mask3 = 0;
      mask4 = 0;
      dayDateReg = decToBcd(dayOrWeekday) & 0x3F;
      secReg = decToBcd(second);
      minReg = decToBcd(minute);
      hourReg = decToBcd(hour);
      break;

    case ALARM_YEARLY:
      mask1 = 0;
      mask2 = 0;
      mask3 = 0;
      mask4 = 0;
      dayDateReg = decToBcd(dayOrWeekday) & 0x3F;
      secReg = decToBcd(second);
      minReg = decToBcd(minute);
      hourReg = decToBcd(hour);
      break;
  }

  if (mask1) secReg |= 0x80;
  if (mask2) minReg |= 0x80;
  if (mask3) hourReg |= 0x80;
  if (mask4) dayDateReg |= 0x80;

  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(REG_ALARM1_SECONDS);
  Wire.write(secReg);
  Wire.write(minReg);
  Wire.write(hourReg);
  Wire.write(dayDateReg);
  Wire.endTransmission();

  uint8_t controlReg = 0;
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(REG_CONTROL);
  Wire.endTransmission();

  Wire.requestFrom(DS3231_ADDRESS, (uint8_t)1);
  controlReg = Wire.read();

  controlReg |= 0x01;
  controlReg |= 0x04;
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(REG_CONTROL);
  Wire.write(controlReg);
  Wire.endTransmission();

  clearAlarm1Flag();
}

void DS3231_RTC::clearAlarm1Flag() {
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(REG_STATUS);
  Wire.endTransmission();

  Wire.requestFrom(DS3231_ADDRESS, (uint8_t)1);
  uint8_t status = Wire.read();

  status &= ~0x01;

  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(REG_STATUS);
  Wire.write(status);
  Wire.endTransmission();
}

void DS3231_RTC::enableAlarmInterrupt(bool enable) {
  uint8_t controlReg = 0;
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(REG_CONTROL);
  Wire.endTransmission();

  Wire.requestFrom(DS3231_ADDRESS, (uint8_t)1);
  controlReg = Wire.read();

  if(enable) {
    controlReg |= 0x05;
  } else {
    controlReg &= ~0x05;
  }

  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(REG_CONTROL);
  Wire.write(controlReg);
  Wire.endTransmission();
}

bool DS3231_RTC::isAlarm1Triggered() {
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(REG_STATUS);
  Wire.endTransmission();

  Wire.requestFrom(DS3231_ADDRESS, (uint8_t)1);
  uint8_t status = Wire.read();

  return (status & 0x01) != 0;
}
