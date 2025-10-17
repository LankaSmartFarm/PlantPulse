
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp_pm.h"
#include "dataLogging.h"
#include "mqtt.h"
#include "storage/FS.h"
#include "myRtc.h"
#include "machinConfig.h"
#include "BLE/ble50_sec_gatts.h"

extern QueueHandle_t modbus_payload_queue;
extern TaskHandle_t dataLoggingTask_Handle;
extern QueueHandle_t mqttAckQueue;
extern TaskHandle_t rtcTask_Handle;
extern TaskHandle_t publishHandleTask_handle;
extern TaskHandle_t connectionHandleTask_handle;
extern TaskHandle_t soilBleTaskHandle;
extern TaskHandle_t pingBleTaskHandle;

extern TaskFunction_t receiveHandleTask_handle;
extern networkState currentStatus;

static void configure_dynamic_frequency(void);
static void reduce_cpu_frequency(void);

void HALT()
{
    while (1)
    {
    }
}

void app_main(void)
{
    // configure_dynamic_frequency();
    #if defined(GSM_DEVICE)

    modbus_payload_queue = xQueueCreate(5, sizeof(M_payload_t));
    mqttAckQueue = xQueueCreate(5, sizeof(char[128])); // Store file paths only

    if (modbus_payload_queue == NULL || mqttAckQueue == NULL)
    {
        ESP_LOGE("modbus_payload_queue", "Failed to create");
        ESP_LOGE("mqttAckQueue", "Failed to create");
        esp_restart();
    }
    #endif
    if (init_fatfs() == ESP_OK)
    {
        setup_directories();
    }
    else
    {
        ESP_LOGE("FAT", "Failed to mount FATFS");
    }
#if !defined(GSM_DEVICE)
    BLEStart();
#endif
#if defined(GSM_DEVICE)
    isrInit();
    rtcInit();
    initUART();
#endif
    RS_485_Init();
    initTask();
#if defined(GSM_DEVICE)
    struct tm now;
#endif
    // Set time once, then comment this line
    while (1)
    {

        vTaskDelay(pdMS_TO_TICKS(1000));
#if defined(GSM_DEVICE)

        if (currentStatus >= MQTT_STATE_MQTT_CONNECTED)
            checkPendingLogs();
        getTime(&now);
        printf("Time: %02d:%02d:%02d Date: %02d-%02d-%04d\n",
               now.tm_hour, now.tm_min, now.tm_sec,
               now.tm_mday, now.tm_mon + 1, now.tm_year + 1900);
#endif

        // if (sleepEnable == SLEEP)
        // {
        //     reduce_cpu_frequency();
        //     enter_deep_sleep();
        // }
    }
}
void initTask(void)
{
#if defined(GSM_DEVICE)
    xTaskCreatePinnedToCore(connectionHandleTask, "connectionHandleTask", 1024 * 4, NULL, 2, &connectionHandleTask_handle, 0);
    xTaskCreatePinnedToCore(receiveHandleTask, "receiveHandleTask", 1024 * 6, NULL, 5, &receiveHandleTask_handle, 1);
    xTaskCreatePinnedToCore(publishHandleTask, "publishHandleTask", 1024 * 4, NULL, 2, &publishHandleTask_handle, 0);
    xTaskCreatePinnedToCore(dataLoggingHandleTask, "data_collect_task", 1024 * 4, NULL, 2, &dataLoggingTask_Handle, 0);
    xTaskCreatePinnedToCore(rtcTask, "rtcTask", 1024 * 4, NULL, 2, &rtcTask_Handle, 1);
#else
    xTaskCreate(soil_ble_task, "soil_ble_task", 4096, NULL, 5, &soilBleTaskHandle);
    xTaskCreate(ping_ble_task, "ping_ble_task", 4096, NULL, 5, &pingBleTaskHandle);

#endif
}

static void configure_dynamic_frequency(void)
{
    esp_pm_config_esp32s3_t pm_config = {
        .max_freq_mhz = 240,        // Maximum frequency
        .min_freq_mhz = 80,         // Minimum frequency
        .light_sleep_enable = false // Keep in active mode
    };
    esp_err_t ret = esp_pm_configure(&pm_config);
    if (ret == ESP_OK)
    {
        ESP_LOGI("Frequency", "Dynamic CPU frequency scaling configured.");
    }
    else
    {
        ESP_LOGE("Frequency", "Failed to configure CPU frequency: %s", esp_err_to_name(ret));
    }
}

static void reduce_cpu_frequency(void)
{
    // ESP_LOGE("Frequency", "delete all task");
    esp_pm_dump_locks(stdout);

    // Delete all tasks except the current one
    vTaskDelay(pdMS_TO_TICKS(100)); // Allow time for tasks to clean
    vTaskSuspendAll();
    esp_pm_config_esp32s3_t pm_config = {
        .max_freq_mhz = 80, // Set both min and max to 80 MHz to reduce power
        .min_freq_mhz = 4,
        .light_sleep_enable = false};
    esp_err_t ret = esp_pm_configure(&pm_config);
    vTaskDelay(pdMS_TO_TICKS(10)); // Allow time for frequency update

    if (ret == ESP_OK)
    {
        // ESP_LOGI("Frequency", "Dynamic CPU frequency scaling configured.");
    }
    else
    {
        // ESP_LOGE("Frequency", "Failed to configure CPU frequency: %s", esp_err_to_name(ret));
    }
    esp_pm_dump_locks(stdout);
    //int cpu_freq_mhz = esp_clk_cpu_freq() / 1000000;
   // ESP_LOGI("CPU Monitor", "Current CPU frequency: %d MHz", cpu_freq_mhz);
    // HALT
}

static void enter_deep_sleep(void)
{
    ESP_LOGI("DEEP_SLEEP", "Going to deep sleep...");
// gpio_pullup_en(PIR);  // Enable pull-up resistor
#if defined(GSM_DEVICE)
    esp_sleep_enable_ext0_wakeup(INT_PIN, 0); // Wake-up when button is pressed (LOW)
#endif
    // Enter deep sleep
    esp_deep_sleep_start();
}