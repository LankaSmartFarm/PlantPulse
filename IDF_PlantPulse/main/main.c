
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dataLogging.h"
#include "mqtt.h"
#include "storage/FS.h"

extern QueueHandle_t modbus_payload_queue;
extern TaskHandle_t dataLoggingTask_Handle;
extern QueueHandle_t mqttAckQueue;

void HALT()
{
    while (1)
    {
    }
}

void app_main(void)
{

    modbus_payload_queue = xQueueCreate(5, sizeof(M_payload_t));
    mqttAckQueue = xQueueCreate(5, sizeof(char[128])); // Store file paths only

    if (modbus_payload_queue == NULL || mqttAckQueue == NULL)
    {
        ESP_LOGE("modbus_payload_queue", "Failed to create");
        ESP_LOGE("mqttAckQueue", "Failed to create");
        esp_restart();
    }
    if (init_fatfs() == ESP_OK)
    {
        setup_directories();
    }
    else
    {
        ESP_LOGE("FAT", "Failed to mount FATFS");
    }

    initGPIO();
    initUART();
    initTask();
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
        checkPendingLogs();
        // ESP_LOGI("MAIN", "NOTIFY TO LOGGING DATA");
        // xTaskNotifyGive(dataLoggingTask_Handle);
    }
}