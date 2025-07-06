
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sim800l.h"
#include "mqtt_client.h"


#include "freertos/semphr.h"

extern SemaphoreHandle_t uart_mutex;


void app_main(void) {


    sim800l_gpio_init();
    sim800l_uart_init();
    uart_mutex = xSemaphoreCreateMutex();
    if (uart_mutex == NULL) {
        ESP_LOGE("MAIN", "Failed to create UART mutex!");
        return;
    }

    mqtt_sim800l_start();
}
