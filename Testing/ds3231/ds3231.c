#include "ds3231.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define I2C_FREQ_HZ 100000

static uint8_t dec_to_bcd(uint8_t val) {
    return (val / 10 * 16 + val % 10);
}

static uint8_t bcd_to_dec(uint8_t val) {
    return (val / 16 * 10 + val % 16);
}

esp_err_t ds3231_init(i2c_port_t i2c_num, gpio_num_t sda_pin, gpio_num_t scl_pin) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = scl_pin,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    esp_err_t res = i2c_param_config(i2c_num, &conf);
    if (res != ESP_OK) return res;
    return i2c_driver_install(i2c_num, conf.mode, 0, 0, 0);
}

esp_err_t ds3231_set_time(i2c_port_t i2c_num, const rtc_time_t *time) {
    uint8_t buffer[8];
    buffer[0] = 0x00; 
    buffer[1] = dec_to_bcd(time->seconds);
    buffer[2] = dec_to_bcd(time->minutes);
    buffer[3] = dec_to_bcd(time->hours);
    buffer[4] = dec_to_bcd(time->day);
    buffer[5] = dec_to_bcd(time->date);
    buffer[6] = dec_to_bcd(time->month);
    buffer[7] = dec_to_bcd(time->year % 100);

    return i2c_master_write_to_device(i2c_num, DS3231_I2C_ADDR, buffer, sizeof(buffer), pdMS_TO_TICKS(1000));
}

esp_err_t ds3231_get_time(i2c_port_t i2c_num, rtc_time_t *time) {
    uint8_t reg = 0x00;
    uint8_t data[7];

    esp_err_t res = i2c_master_write_read_device(i2c_num, DS3231_I2C_ADDR, &reg, 1, data, 7, pdMS_TO_TICKS(1000));
    if (res != ESP_OK) return res;

    time->seconds = bcd_to_dec(data[0] & 0x7F);
    time->minutes = bcd_to_dec(data[1]);
    time->hours = bcd_to_dec(data[2] & 0x3F);
    time->day = bcd_to_dec(data[3]);
    time->date = bcd_to_dec(data[4]);
    time->month = bcd_to_dec(data[5] & 0x1F);
    time->year = 2000 + bcd_to_dec(data[6]);

    return ESP_OK;
}
