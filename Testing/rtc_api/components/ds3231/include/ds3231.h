#ifndef DS3231_H
#define DS3231_H

#include <esp_err.h>
#include <driver/i2c.h>
#include <time.h>

// DS3231 I2C address
#define DS3231_ADDRESS 0x68

// DS3231 register addresses
#define DS3231_REG_SECONDS 0x00
#define DS3231_REG_ALARM1_SECONDS 0x07
#define DS3231_REG_ALARM2_MINUTES 0x0B
#define DS3231_REG_CONTROL 0x0E
#define DS3231_REG_STATUS 0x0F

// Alarm modes
typedef enum {
    DS3231_ALARM1_EVERY_SECOND, // Alarm every second
    DS3231_ALARM1_SECONDS,      // Alarm when seconds match
    DS3231_ALARM1_MINUTES,      // Alarm when minutes and seconds match
    DS3231_ALARM1_HOURS,        // Alarm when hours, minutes, and seconds match
    DS3231_ALARM1_DATE,         // Alarm when date, hours, minutes, and seconds match
    DS3231_ALARM1_DAY           // Alarm when day, hours, minutes, and seconds match
} ds3231_alarm1_rate_t;

typedef enum {
    DS3231_ALARM2_EVERY_MINUTE, // Alarm every minute
    DS3231_ALARM2_MINUTES,      // Alarm when minutes match
    DS3231_ALARM2_HOURS,        // Alarm when hours and minutes match
    DS3231_ALARM2_DATE,         // Alarm when date, hours, and minutes match
    DS3231_ALARM2_DAY           // Alarm when day, hours, and minutes match
} ds3231_alarm2_rate_t;

// Alarm configuration structure
typedef struct {
    struct tm time;              // Time for the alarm
    ds3231_alarm1_rate_t rate1;  // Alarm 1 rate
    ds3231_alarm2_rate_t rate2;  // Alarm 2 rate
    bool enabled;                // Alarm enabled status
} ds3231_alarm_t;

// I2C device descriptor
typedef struct {
    i2c_port_t port;
    i2c_config_t cfg;
    uint8_t addr;
} ds3231_dev_t;

// Function prototypes
esp_err_t ds3231_init(ds3231_dev_t *dev, i2c_port_t port, gpio_num_t sda_pin, gpio_num_t scl_pin);
esp_err_t ds3231_set_time(ds3231_dev_t *dev, struct tm *time);
esp_err_t ds3231_get_time(ds3231_dev_t *dev, struct tm *time);
esp_err_t ds3231_set_alarm(ds3231_dev_t *dev, ds3231_alarm_t *alarm1, ds3231_alarm_t *alarm2);
esp_err_t ds3231_enable_alarm(ds3231_dev_t *dev, uint8_t alarm_num, bool enable);
esp_err_t ds3231_clear_alarm(ds3231_dev_t *dev, uint8_t alarm_num);
esp_err_t ds3231_check_alarm(ds3231_dev_t *dev, uint8_t alarm_num, bool *triggered);
esp_err_t ds3231_enable_sqw(ds3231_dev_t *dev, bool enable);
esp_err_t ds3231_init_wakeup(ds3231_dev_t *dev, gpio_num_t sqw_pin);

#endif