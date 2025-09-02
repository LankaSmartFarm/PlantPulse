#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_sleep.h>
#include <esp_log.h>
#include "ds3231.h"

#define WAKE_GPIO GPIO_NUM_10  // DS3231 INT pin

static const char *TAG = "MAIN";
RTC_DATA_ATTR period_t current_period;
RTC_DATA_ATTR alarm_config_t current_config;
RTC_DATA_ATTR bool is_configured = false;


void perform_task(void) {
    ESP_LOGI(TAG, "Performing task!");
}

void app_main(void) 
{
    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
    ds3231_init();

    if (wakeup_cause == ESP_SLEEP_WAKEUP_EXT0) {
        clear_alarm_flag();  // Reset INT pin

        Time current_time;
        get_time(&current_time);

        bool do_task = true;
        if (current_period == YEARLY) {
            if ((current_time.month + 1) != current_config.month) {
                do_task = false;
            }
        }

        if (do_task) {
            perform_task();    
        }

    } else if (!is_configured) {

        // Initial setup (first boot or reset)
        // Example: Set current time to Sep 2, 2025, 12:00:00, Tuesday (wday=2)
        Time initial_time = {
            .seconds = 0,
            .minutes= 0,
            .hours = 12,
            .day = 2,
            .month = 8,  // September (0-11)
            .year = 125,  // 2025 - 1900
            .wday = 2  // Tuesday (0-6)
        };
        set_time(&initial_time);

        // Example: Set Hourly alarm 
        alarm_config_t config = {
            .sec = 0,
            .min = 1,
            .hour = 0,
            .dow = 0,  // Unused
            .date = 0,
            .month = 0
        };
        current_period = HOURLY;
        rtc_set_alarm(current_period, &config);
        current_config = config;
        is_configured = true;

        ESP_LOGI(TAG, "Initial configuration done.");
    }

    // Configure wake-up and sleep
    esp_sleep_enable_ext0_wakeup(WAKE_GPIO, 0);  // Wake on low level
    // Optional: Enable pull-up on wake GPIO
    gpio_pullup_en(WAKE_GPIO);

    ESP_LOGI(TAG, "Going to sleep!");
    esp_deep_sleep_start();
    
}