#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ds3231.h"

#define I2C_PORT I2C_NUM_0
#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

void app_main(void) {
    ds3231_init(I2C_PORT, SDA_PIN, SCL_PIN);

    rtc_time_t time = {
        .seconds = 0,
        .minutes = 30,
        .hours = 15,
        .day = 4,
        .date = 17,
        .month = 7,
        .year = 2025
    };

    ds3231_set_time(I2C_PORT, &time);

    while (1) {
        rtc_time_t now;
        ds3231_get_time(I2C_PORT, &now);
        printf("Time: %02d:%02d:%02d | Date: %04d-%02d-%02d (DOW: %d)\n",
               now.hours, now.minutes, now.seconds,
               now.year, now.month, now.date, now.day);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
