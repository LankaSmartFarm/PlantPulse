#ifndef __DS3231_H__
#define __DS3231_H__

#include <stdint.h>
#include "driver/i2c.h"

#define DS3231_I2C_ADDR  0x68

typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t day;   
    uint8_t date;  
    uint8_t month;
    uint16_t year;
} rtc_time_t;

esp_err_t ds3231_init(i2c_port_t i2c_num, gpio_num_t sda_pin, gpio_num_t scl_pin);
esp_err_t ds3231_set_time(i2c_port_t i2c_num, const rtc_time_t *time);
esp_err_t ds3231_get_time(i2c_port_t i2c_num, rtc_time_t *time);

#endif
