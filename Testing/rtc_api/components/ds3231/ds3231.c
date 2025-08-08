#include "ds3231.h"
#include <string.h>
#include <esp_log.h>
#include <esp_sleep.h>

static const char *TAG = "DS3231";

// Helper function to convert decimal to BCD
static uint8_t dec_to_bcd(uint8_t dec) {
    return ((dec / 10) << 4) | (dec % 10);
}

// Helper function to convert BCD to decimal
static uint8_t bcd_to_dec(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}


// Initialize I2C and DS3231
esp_err_t ds3231_init(ds3231_dev_t *dev, i2c_port_t port, gpio_num_t sda_pin, gpio_num_t scl_pin) {
    ESP_LOGI(TAG, "Initializing DS3231 on I2C port %d", port);
    dev->port = port;
    dev->addr = DS3231_ADDRESS;
    dev->cfg.mode = I2C_MODE_MASTER;
    dev->cfg.sda_io_num = sda_pin;
    dev->cfg.scl_io_num = scl_pin;
    dev->cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    dev->cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    dev->cfg.master.clk_speed = 100000; // 100 kHz
i2c_config_t cfg;
i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_PORT_NUM_0,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .glitch_ignore_cnt = 7,
};
i2c_master_bus_handle_t bus_handle;

    esp_err_t ret = i2c_param_config(port, &dev->cfg);
    if (ret != ESP_OK) return ret;

    ret = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) return ret;

    // Disable 32kHz output and set SQW for alarms
    uint8_t control = 0x00; // INTCN = 1, alarms enabled
    return i2c_master_write_to_device(port, DS3231_ADDRESS, (uint8_t[]){DS3231_REG_CONTROL, control}, 2, 1000 / portTICK_PERIOD_MS);
}

// Set time on DS3231
esp_err_t ds3231_set_time(ds3231_dev_t *dev, struct tm *time) {
    uint8_t data[7] = {
        dec_to_bcd(time->tm_sec),
        dec_to_bcd(time->tm_min),
        dec_to_bcd(time->tm_hour),
        dec_to_bcd(time->tm_wday + 1), // 1-7 (Sunday-Saturday)
        dec_to_bcd(time->tm_mday),
        dec_to_bcd(time->tm_mon + 1),  // 1-12
        dec_to_bcd(time->tm_year - 100) // Year since 1900
    };
    return i2c_master_write_to_device(dev->port, dev->addr, (uint8_t[]){DS3231_REG_SECONDS, data[0], data[1], data[2], data[3], data[4], data[5], data[6]}, 8, 1000 / portTICK_PERIOD_MS);
}

// Get time from DS3231
esp_err_t ds3231_get_time(ds3231_dev_t *dev, struct tm *time) {
    uint8_t data[7];
    esp_err_t ret = i2c_master_write_read_device(dev->port, dev->addr, (uint8_t[]){DS3231_REG_SECONDS}, 1, data, 7, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;

    time->tm_sec = bcd_to_dec(data[0]);
    time->tm_min = bcd_to_dec(data[1]);
    time->tm_hour = bcd_to_dec(data[2]);
    time->tm_wday = bcd_to_dec(data[3]) - 1;
    time->tm_mday = bcd_to_dec(data[4]);
    time->tm_mon = bcd_to_dec(data[5]) - 1;
    time->tm_year = bcd_to_dec(data[6]) + 100;
    return ESP_OK;
}

// Set alarms (Alarm 1 and Alarm 2)
esp_err_t ds3231_set_alarm(ds3231_dev_t *dev, ds3231_alarm_t *alarm1, ds3231_alarm_t *alarm2) {
    esp_err_t ret;
    uint8_t data[4];

    // Alarm 1
    if (alarm1 && alarm1->enabled) {
        data[0] = dec_to_bcd(alarm1->time.tm_sec);
        data[1] = dec_to_bcd(alarm1->time.tm_min);
        data[2] = dec_to_bcd(alarm1->time.tm_hour);
        data[3] = dec_to_bcd(alarm1->time.tm_mday);
        if (alarm1->rate1 == DS3231_ALARM1_DAY) data[3] |= 0x40; // Set A1M4 for day of week
        switch (alarm1->rate1) {
            case DS3231_ALARM1_EVERY_SECOND: data[0] |= 0x80; data[1] |= 0x80; data[2] |= 0x80; data[3] |= 0x80; break;
            case DS3231_ALARM1_SECONDS: data[1] |= 0x80; data[2] |= 0x80; data[3] |= 0x80; break;
            case DS3231_ALARM1_MINUTES: data[2] |= 0x80; data[3] |= 0x80; break;
            case DS3231_ALARM1_HOURS: data[3] |= 0x80; break;
            default: break;
        }
        ret = i2c_master_write_to_device(dev->port, dev->addr, (uint8_t[]){DS3231_REG_ALARM1_SECONDS, data[0], data[1], data[2], data[3]}, 5, 1000 / portTICK_PERIOD_MS);
        if (ret != ESP_OK) return ret;
    }

    // Alarm 2
    if (alarm2 && alarm2->enabled) {
        data[0] = dec_to_bcd(alarm2->time.tm_min);
        data[1] = dec_to_bcd(alarm2->time.tm_hour);
        data[2] = dec_to_bcd(alarm2->time.tm_mday);
        if (alarm2->rate2 == DS3231_ALARM2_DAY) data[2] |= 0x40; // Set A2M4 for day of week
        switch (alarm2->rate2) {
            case DS3231_ALARM2_EVERY_MINUTE: data[0] |= 0x80; data[1] |= 0x80; data[2] |= 0x80; break;
            case DS3231_ALARM2_MINUTES: data[1] |= 0x80; data[2] |= 0x80; break;
            case DS3231_ALARM2_HOURS: data[2] |= 0x80; break;
            default: break;
        }
        ret = i2c_master_write_to_device(dev->port, dev->addr, (uint8_t[]){DS3231_REG_ALARM2_MINUTES, data[0], data[1], data[2]}, 4, 1000 / portTICK_PERIOD_MS);
        if (ret != ESP_OK) return ret;
    }

    // Enable alarms in control register
    uint8_t control;
    ret = i2c_master_write_read_device(dev->port, dev->addr, (uint8_t[]){DS3231_REG_CONTROL}, 1, &control, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;
    control |= (alarm1 && alarm1->enabled) ? 0x01 : 0; // A1IE
    control |= (alarm2 && alarm2->enabled) ? 0x02 : 0; // A2IE
    control |= 0x04; // INTCN = 1 for interrupt output
    return i2c_master_write_to_device(dev->port, dev->addr, (uint8_t[]){DS3231_REG_CONTROL, control}, 2, 1000 / portTICK_PERIOD_MS);
}

// Enable or disable alarm
esp_err_t ds3231_enable_alarm(ds3231_dev_t *dev, uint8_t alarm_num, bool enable) {
    uint8_t control;
    esp_err_t ret = i2c_master_write_read_device(dev->port, dev->addr, (uint8_t[]){DS3231_REG_CONTROL}, 1, &control, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;

    if (alarm_num == 1) {
        control = enable ? (control | 0x01) : (control & ~0x01);
    } else if (alarm_num == 2) {
        control = enable ? (control | 0x02) : (control & ~0x02);
    }
    return i2c_master_write_to_device(dev->port, dev->addr, (uint8_t[]){DS3231_REG_CONTROL, control}, 2, 1000 / portTICK_PERIOD_MS);
}

// Clear alarm flag
esp_err_t ds3231_clear_alarm(ds3231_dev_t *dev, uint8_t alarm_num) {
    uint8_t status;
    esp_err_t ret = i2c_master_write_read_device(dev->port, dev->addr, (uint8_t[]){DS3231_REG_STATUS}, 1, &status, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;

    if (alarm_num == 1) {
        status &= ~0x01; // Clear A1F
    } else if (alarm_num == 2) {
        status &= ~0x02; // Clear A2F
    }
    return i2c_master_write_to_device(dev->port, dev->addr, (uint8_t[]){DS3231_REG_STATUS, status}, 2, 1000 / portTICK_PERIOD_MS);
}

// Check if alarm has triggered
esp_err_t ds3231_check_alarm(ds3231_dev_t *dev, uint8_t alarm_num, bool *triggered) {
    uint8_t status;
    esp_err_t ret = i2c_master_write_read_device(dev->port, dev->addr, (uint8_t[]){DS3231_REG_STATUS}, 1, &status, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;

    *triggered = (alarm_num == 1) ? (status & 0x01) : (status & 0x02);
    return ESP_OK;
}

// Enable/disable SQW output
esp_err_t ds3231_enable_sqw(ds3231_dev_t *dev, bool enable) {
    uint8_t control;
    esp_err_t ret = i2c_master_write_read_device(dev->port, dev->addr, (uint8_t[]){DS3231_REG_CONTROL}, 1, &control, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;

    control = enable ? (control | 0x04) : (control & ~0x04); // INTCN bit
    return i2c_master_write_to_device(dev->port, dev->addr, (uint8_t[]){DS3231_REG_CONTROL, control}, 2, 1000 / portTICK_PERIOD_MS);
}

// Initialize ESP32 for deep sleep wake-up via SQW
esp_err_t ds3231_init_wakeup(ds3231_dev_t *dev, gpio_num_t sqw_pin) {
    esp_err_t ret = ds3231_enable_sqw(dev, true);
    if (ret != ESP_OK) return ret;

    // Configure SQW pin as input with pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << sqw_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_LOW_LEVEL
    };
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) return ret;

    // Enable external wake-up
    ret = esp_sleep_enable_ext0_wakeup(sqw_pin, 0); // Trigger on LOW
    return ret;
}