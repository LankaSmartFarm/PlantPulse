
#include <string.h>
#include "dataLogging.h"

QueueHandle_t modbus_payload_queue;
TaskHandle_t dataLoggingTask_Handle= NULL;

M_payload_t build_modbus_payload(void) {
    M_payload_t payload;
    memset(&payload, 0, sizeof(payload));

    // 1. Timestamp (Unix or custom function)
    // payload.timestamp = get_unix_timestamp();  // e.g., from RTC task

    // 2. Device ID (6 bytes)
    // get_device_id(payload.device_id);  // e.g., ESP32 MAC address

    // 3. Topic (max 20 bytes)
    // strncpy(payload.topic, "sensor/data", sizeof(payload.topic));

    // 4. Battery level (mV)
    // payload.battery_mv = get_battery_mv();  // Your custom ADC read

    // 5. Read all 20 Modbus sensors
    for (int i = 0; i < SENSOR_COUNT; i++) {
        uint8_t slave_id = SENSOR_SLAVE_START + i;
        uint8_t tx_buf[8] = {0};
        uint8_t rx_buf[64] = {0};
        bool success = false;

        // turn on the sensor here

        for (int attempt = 0; attempt < 5; attempt++) {
            MB_rtu_send(slave_id, SENSOR_FUNC_CODE, SENSOR_REG_ADDR, SENSOR_REG_COUNT, tx_buf, 0);
            int recv_len = MB_rtu_receive(rx_buf, sizeof(rx_buf));

            int expected_len = 5 + SENSOR_REG_COUNT * 2;  // 1 addr + 1 func + 1 byteCount + 2*reg + 2 CRC
            if (recv_len >= expected_len) {
                uint16_t recv_crc = (rx_buf[recv_len - 2]) | (rx_buf[recv_len - 1] << 8);
                uint16_t calc_crc = crc16(rx_buf, recv_len - 2);

                if (recv_crc != calc_crc) {
                    ESP_LOGW("MODBUS", "CRC mismatch from Sensor ID %d (attempt %d)", slave_id, attempt + 1);
                    continue;
                }

                // Parse 7 values: pH, moisture, temperature, ec, N, P, K
                payload.sensor_data[i].ph           = (rx_buf[3] << 8) | rx_buf[4];
                payload.sensor_data[i].moisture     = (rx_buf[5] << 8) | rx_buf[6];
                payload.sensor_data[i].temperature  = (int16_t)((rx_buf[7] << 8) | rx_buf[8]);
                payload.sensor_data[i].conductivity = (rx_buf[9] << 8) | rx_buf[10];
                payload.sensor_data[i].nitrogen     = (rx_buf[11] << 8) | rx_buf[12];
                payload.sensor_data[i].phosphorus   = (rx_buf[13] << 8) | rx_buf[14];
                payload.sensor_data[i].potassium    = (rx_buf[15] << 8) | rx_buf[16];

                success = true;
                break;
            } else {
                ESP_LOGW("MODBUS", "Sensor ID %d response error (attempt %d)", slave_id, attempt + 1);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (!success) {
            memset(&payload.sensor_data[i], 0xFF, sizeof(payload.sensor_data[i]));
            ESP_LOGW("MODBUS", "Sensor ID %d failed after 5 attempts", slave_id);
        }
    }

    // 6. Reserved bytes (4 bytes can be any future use)
    payload.reserved[0] = 0xFF;
    payload.reserved[1] = 0xFF;
    payload.reserved[2] = 0xFF;
    payload.reserved[3] = 0xFF;

    // 7. CRC (over the whole payload except this CRC field)
    payload.crc = crc16((uint8_t *)&payload, sizeof(payload) - 2);

    return payload;
}



void dataLoggingHandleTask(void *pvParameters)
{
    M_payload_t payload;

    while (1)
    {
        // Wait for notification from RTC or triggering task
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // ESP_LOGI("MODBUS_TASK", "Collecting Modbus data...");
        // payload = build_modbus_payload();

        //-------------------dammy data------------------------------------------------------

        ESP_LOGI("MODBUS_TASK", "Collecting dummy Modbus data...");
        // --- Fill timestamp ---
        payload.timestamp = esp_timer_get_time(); // microseconds since boot
        // --- Dummy device ID (can use MAC in future) ---
        uint8_t dummy_mac[6] = { 0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33 };
        memcpy(payload.device_id, dummy_mac, 6);
        // --- Topic ---
        strncpy(payload.topic, "soil/sensor", sizeof(payload.topic));
        // --- Dummy battery voltage ---
        payload.battery_mv = 3700;
        // --- Dummy data for 20 sensors ---
        for (int i = 0; i < 1; i++) {
            payload.sensor_data[i].ph = 700 + i;             // e.g., pH = 7.00
            payload.sensor_data[i].moisture = 350 + i;
            payload.sensor_data[i].temperature = 250 + i;    // e.g., 25.0°C
            payload.sensor_data[i].conductivity = 500 + i;
            payload.sensor_data[i].nitrogen = 20 + i;
            payload.sensor_data[i].phosphorus = 10 + i;
            payload.sensor_data[i].potassium = 30 + i;
        }
        // --- Reserved ---
        memset(payload.reserved, 0xFF, sizeof(payload.reserved));
        // --- CRC ---
        payload.crc = crc16((uint8_t*)&payload, sizeof(M_payload_t) - sizeof(payload.crc));
        //------------------dummy end--------------------------------------------------------
        
        if (xQueueSend(modbus_payload_queue, &payload, pdMS_TO_TICKS(100)) != pdPASS) {
            ESP_LOGW("MODBUS_TASK", "Failed to queue Modbus payload");
        } else {
            ESP_LOGI("MODBUS_TASK", "Modbus payload queued for MQTT");
        }
    }
}
