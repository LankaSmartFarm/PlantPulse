
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dataLogging.h"
#include "mqtt.h"

extern QueueHandle_t modbus_payload_queue;
extern TaskHandle_t dataLoggingTask_Handle;
void HALT()
{
    while (1)
    {
    }
}

void app_main(void)
{

    modbus_payload_queue = xQueueCreate(5, sizeof(M_payload_t));

    if (modbus_payload_queue == NULL)
    {
        ESP_LOGE("modbus_payload_queue", "Failed to create");
        esp_restart();
    }
    initGPIO();
    initUART();
    initTask();
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
        // ESP_LOGI("MAIN", "NOTIFY TO LOGGING DATA");
        // xTaskNotifyGive(dataLoggingTask_Handle);
    }
}