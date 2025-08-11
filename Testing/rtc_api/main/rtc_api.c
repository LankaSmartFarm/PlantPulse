#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_sleep.h>
#include <esp_log.h>
#include "ds3231.h"

static const char *TAG = "MAIN";

RTC_DATA_ATTR int boot_count = 0; // Persist across deep sleep

void print_wakeup_reason() {
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    switch (cause) {
        case ESP_SLEEP_WAKEUP_EXT0: ESP_LOGI(TAG, "Wake-up caused by external signal (RTC alarm)"); break;
        default: ESP_LOGI(TAG, "Wake-up not caused by deep sleep: %d", cause); break;
    }
}

void app_main(void) {
    // Increment boot count
    boot_count++;
    ESP_LOGI(TAG, "Boot count: %d", boot_count);

    // Print wake-up reason
    print_wakeup_reason();

    // Initialize DS3231
    ds3231_dev_t dev;
    // ESP_ERROR_CHECK(ds3231_init(&dev, I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22));
    ds3231_init(&dev, I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22);


    // Set initial time (e.g., 2025-08-08 01:00:00, adjust as needed)
    struct tm time = {
        .tm_year = 125, // 2025 - 1900
        .tm_mon = 7,    // August (0-based)
        .tm_mday = 8,
        .tm_hour = 1,
        .tm_min = 19,
        .tm_sec = 0,
        .tm_wday = 5    // Friday (0-based, set for reference)
    };
    // ESP_ERROR_CHECK(ds3231_set_time(&dev, &time));
    ds3231_set_time(&dev, &time);

    // Configure Alarm 1 (every Friday at 01:20:00)
    ds3231_alarm_t alarm1 = {
        .time = {
            .tm_hour = 1,   // 01:20 AM
            .tm_min = 20,
            .tm_sec = 0,
              // Friday (0-based in struct tm, converted to 1-7 in library)
        },
        .rate1 = DS3231_ALARM1_MINUTES, // Trigger on specific day of week
        .enabled = true
    };
    // ds3231_alarm_t alarm1 = {
    // .time = {0}, // Time fields ignored for EVERY_MINUTE
    // .rate1 = DS3231_ALARM1_MINUTES,
    // .enabled = true
    // };


    //ESP_ERROR_CHECK(ds3231_set_alarm(&dev, &alarm1, NULL));
    ds3231_set_alarm(&dev, &alarm1, NULL);
    // Initialize wake-up on SQW pin (GPIO 33)
    //ESP_ERROR_CHECK(ds3231_init_wakeup(&dev, GPIO_NUM_33));
    ds3231_init_wakeup(&dev, GPIO_NUM_33);
    // Perform task (e.g., log current time)
   // ESP_ERROR_CHECK(ds3231_get_time(&dev, &time));
    ds3231_get_time(&dev, &time);
    ESP_LOGI(TAG, "Current time: %04d-%02d-%02d %02d:%02d:%02d (Day: %d)",
             time.tm_year + 1900, time.tm_mon + 1, time.tm_mday,
             time.tm_hour, time.tm_min, time.tm_sec, time.tm_wday + 1);

    // Clear alarm flag
   // ESP_ERROR_CHECK(ds3231_clear_alarm(&dev, 1));
        ds3231_clear_alarm(&dev, 1);
    // Enter deep sleep
    ESP_LOGI(TAG, "Entering deep sleep until next Friday at 01:20:00...");
    esp_deep_sleep_start();
}