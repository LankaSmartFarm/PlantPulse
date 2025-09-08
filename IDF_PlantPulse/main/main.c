
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dataLogging.h"
#include "mqtt.h"
#include "storage/FS.h"
#include "myRtc.h"

extern QueueHandle_t modbus_payload_queue;
extern TaskHandle_t dataLoggingTask_Handle;
extern QueueHandle_t mqttAckQueue;
extern TaskHandle_t rtcTask_Handle;
extern TaskHandle_t publishHandleTask_handle;
extern TaskHandle_t connectionHandleTask_handle;
extern TaskFunction_t receiveHandleTask_handle;
extern networkState currentStatus;
// void HALT()
// {
//     while (1)
//     {
//     }
// }

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
    isrInit();
    rtcInit();
    initUART();
    initTask();
    struct tm now;
    // Set time once, then comment this line
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (currentStatus >= MQTT_STATE_MQTT_CONNECTED)
            checkPendingLogs();
        getTime(&now);
        printf("Time: %02d:%02d:%02d Date: %02d-%02d-%04d\n",
               now.tm_hour, now.tm_min, now.tm_sec,
               now.tm_mday, now.tm_mon + 1, now.tm_year + 1900);
    }
}
void initTask(void)
{
    xTaskCreatePinnedToCore(connectionHandleTask, "connectionHandleTask", 1024 * 4, NULL, 2, &connectionHandleTask_handle, 0);
    xTaskCreatePinnedToCore(receiveHandleTask, "receiveHandleTask", 1024 * 6, NULL, 5, &receiveHandleTask_handle, 1);
    xTaskCreatePinnedToCore(publishHandleTask, "publishHandleTask", 1024 * 4, NULL, 2, &publishHandleTask_handle, 0);
    xTaskCreatePinnedToCore(dataLoggingHandleTask, "data_collect_task", 1024 * 4, NULL, 2, &dataLoggingTask_Handle, 0);
    xTaskCreatePinnedToCore(rtcTask, "rtcTask", 1024 * 4, NULL, 2, &rtcTask_Handle, 1);
}
