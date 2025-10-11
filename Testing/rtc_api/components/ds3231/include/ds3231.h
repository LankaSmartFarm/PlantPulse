#ifndef DS3231_H
#define DS3231_H

#include "driver/i2c.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include <time.h>


#define I2C_MASTER_SDA 8
#define I2C_MASTER_SCL 9
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_NUM I2C_NUM_0
#define DS3231_ADDR 0x68
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

typedef enum 
{
    HOURLY,
    DAILY,
    WEEKLY,
    MONTHLY,
    YEARLY
} period_t;

typedef struct
{
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t day;
    uint8_t wday;
    uint8_t weekly;
    uint8_t month;
    uint8_t year;
} Time;

typedef struct 
{
    int sec;   // 0-59
    int min;   // 0-59
    int hour;  // 0-23
    int dow;   // 1-7 (for weekly)
    int date;  // 1-31 (for monthly/yearly)
    int month; // 1-12 (for yearly)
} alarm_config_t;


void ds3231_init(void);
esp_err_t set_time(Time *time);
esp_err_t get_time(Time *current_time);
void rtc_set_alarm(period_t period, alarm_config_t *config);

// Clear alarm flag (call on wake-up)
void clear_alarm_flag(void);


#endif