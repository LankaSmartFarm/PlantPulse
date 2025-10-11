
#include <string.h>
#include "dataLogging.h"

QueueHandle_t modbus_payload_queue;
TaskHandle_t dataLoggingTask_Handle = NULL;
TaskHandle_t soilBleTaskHandle = NULL;
TaskHandle_t pingBleTaskHandle = NULL;



M_payload_t build_modbus_payload(void)
{
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
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        uint8_t slave_id = SENSOR_SLAVE_START + i;
        uint8_t tx_buf[8] = {0};
        uint8_t rx_buf[64] = {0};
        bool success = false;

        // turn on the sensor here

        for (int attempt = 0; attempt < 5; attempt++)
        {
            MB_rtu_send(slave_id, SOIL8IN1_FUNC_CODE, SOIL8IN1_REG_ADDR, SOIL8IN1_REG_COUNT, tx_buf, 0);
            int recv_len = MB_rtu_receive(rx_buf, sizeof(rx_buf));

            int expected_len = 5 + SOIL8IN1_REG_COUNT * 2; // 1 addr + 1 func + 1 byteCount + 2*reg + 2 CRC
            if (recv_len >= expected_len)
            {
                uint16_t recv_crc = (rx_buf[recv_len - 2]) | (rx_buf[recv_len - 1] << 8);
                uint16_t calc_crc = crc16(rx_buf, recv_len - 2);

                if (recv_crc != calc_crc)
                {
                    ESP_LOGW("MODBUS", "CRC mismatch from Sensor ID %d (attempt %d)", slave_id, attempt + 1);
                    continue;
                }

                // Parse 7 values: pH, moisture, temperature, ec, N, P, K
                payload.sensor_data[i].ph = (rx_buf[3] << 8) | rx_buf[4];
                payload.sensor_data[i].moisture = (rx_buf[5] << 8) | rx_buf[6];
                payload.sensor_data[i].temperature = (int16_t)((rx_buf[7] << 8) | rx_buf[8]);
                payload.sensor_data[i].conductivity = (rx_buf[9] << 8) | rx_buf[10];
                payload.sensor_data[i].nitrogen = (rx_buf[11] << 8) | rx_buf[12];
                payload.sensor_data[i].phosphorus = (rx_buf[13] << 8) | rx_buf[14];
                payload.sensor_data[i].potassium = (rx_buf[15] << 8) | rx_buf[16];

                success = true;
                break;
            }
            else
            {
                ESP_LOGW("MODBUS", "Sensor ID %d response error (attempt %d)", slave_id, attempt + 1);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (!success)
        {
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
        payload = build_modbus_payload();

        //-------------------dammy data------------------------------------------------------

        ESP_LOGI("MODBUS_TASK", "Collecting dummy Modbus data...");
        // --- Fill timestamp ---
        payload.timestamp = esp_timer_get_time(); // microseconds since boot
        // --- Dummy device ID (can use MAC in future) ---
        uint8_t dummy_mac[6] = {0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33};
        memcpy(payload.device_id, dummy_mac, 6);
        // --- Topic ---
        strncpy(payload.topic, "soil/sensor", sizeof(payload.topic));
        // --- Dummy battery voltage ---
        payload.battery_mv = 3700;
        // --- Dummy data for 20 sensors ---
        for (int i = 0; i < 1; i++)
        {
            payload.sensor_data[i].ph = 700 + i; // e.g., pH = 7.00
            payload.sensor_data[i].moisture = 350 + i;
            payload.sensor_data[i].temperature = 250 + i; // e.g., 25.0°C
            payload.sensor_data[i].conductivity = 500 + i;
            payload.sensor_data[i].nitrogen = 20 + i;
            payload.sensor_data[i].phosphorus = 10 + i;
            payload.sensor_data[i].potassium = 30 + i;
        }
        // --- Reserved ---
        memset(payload.reserved, 0xFF, sizeof(payload.reserved));
        // --- CRC ---
        payload.crc = crc16((uint8_t *)&payload, sizeof(M_payload_t) - sizeof(payload.crc));
        //------------------dummy end--------------------------------------------------------

        if (xQueueSend(modbus_payload_queue, &payload, pdMS_TO_TICKS(100)) != pdPASS)
        {
            ESP_LOGW("MODBUS_TASK", "Failed to queue Modbus payload");
        }
        else
        {
            ESP_LOGI("MODBUS_TASK", "Modbus payload queued for MQTT");
        }
    }
}

soil8in1_data_t build_soil8in1_data(void)
{
    soil8in1_data_t data;
    memset(&data, 0, sizeof(data));

    uint8_t tx_buf[8] = {0};
    uint8_t rx_buf[64] = {0};
    bool success = false;

    for (int attempt = 0; attempt < 5; attempt++)
    {
        // Send Modbus request
        MB_rtu_send(SOIL8IN1_SLAVE_ID, SOIL8IN1_FUNC_CODE, SOIL8IN1_REG_ADDR, SOIL8IN1_REG_COUNT, tx_buf, 0);

        // Receive Modbus response
        int recv_len = MB_rtu_receive(rx_buf, sizeof(rx_buf));
        int expected_len = 5 + SOIL8IN1_REG_COUNT * 2; // 1 addr + 1 func + 1 byteCount + (2 * reg count) + 2 CRC

        if (recv_len >= expected_len)
        {
            uint16_t recv_crc = (rx_buf[recv_len - 2]) | (rx_buf[recv_len - 1] << 8);
            uint16_t calc_crc = crc16(rx_buf, recv_len - 2);

            if (recv_crc != calc_crc)
            {
                ESP_LOGW("SOIL8IN1", "CRC mismatch (attempt %d)", attempt + 1);
                continue;
            }

            // Parse the 8 registers (each 2 bytes)
            data.temperature = (rx_buf[3] << 8) | rx_buf[4]; // °C × 100
            data.humidity = (rx_buf[5] << 8) | rx_buf[6];    // % × 100
            data.ec = (rx_buf[7] << 8) | rx_buf[8];          // µS/cm
            data.ph = (rx_buf[9] << 8) | rx_buf[10];         // ×100
            data.nitrogen = (rx_buf[11] << 8) | rx_buf[12];
            data.phosphorus = (rx_buf[13] << 8) | rx_buf[14];
            data.potassium = (rx_buf[15] << 8) | rx_buf[16];
            data.soil_moisture = (rx_buf[17] << 8) | rx_buf[18]; // % × 100

            success = true;
            break;
        }
        else
        {
            ESP_LOGW("SOIL8IN1", "Invalid response length (attempt %d)", attempt + 1);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (!success)
    {
        // Mark data invalid if all attempts failed
        memset(&data, 0xFF, sizeof(data));
        ESP_LOGW("SOIL8IN1", "Failed to read soil sensor after 5 attempts");
    }

    return data;
}

/**
 * @brief Builds a complete soil BLE data packet (64 bytes)
 * @return soil_packet_t - fully ready packet with CRC
 */
void build_soil_packet(soil_packet_t *packet)
{
    memset(&packet, 0, sizeof(packet));

    // 1️⃣ Data type for identification
    packet->data_type = SOIL_PACKET_TYPE;

    // 2️⃣ Device ID (6 bytes)
    // get_device_id(packet.device_id);

    // 3️⃣ Read 8-in-1 soil sensor via Modbus
    packet->soil8in1 = build_soil8in1_data();

    // 4️⃣ Reserved bytes (filled with 0xFF)
    memset(packet->reserved, 0xFF, sizeof(packet->reserved));

    // 5️⃣ CRC (excluding the CRC field itself)
    packet->crc = crc16((uint8_t *)&packet, sizeof(packet) - sizeof(packet->crc));

    ESP_LOGI("SOIL_PACKET", "Soil packet built successfully, CRC=0x%04X", packet->crc);
}


ping_packet_t build_ping_packet(void)
{
    ping_packet_t pkt;
    memset(&pkt, 0xFF, sizeof(pkt)); // Fill all with 0xFF initially

    //pkt.packet_type = PING_PACKET_TYPE;

    // 1️⃣ Device ID (replace with your own MAC getter)
    // get_device_id(pkt.device_id);   // Example: esp_read_mac()

    // 2️⃣ BLE RSSI (you can track last RSSI)
    // pkt.ble_rssi = get_ble_rssi();  // Or use a stored variable

    // 3️⃣ Battery level
    // pkt.battery_level = get_battery_mv();  // e.g., 3850 mV

    // 4️⃣ Charge indicator
    //pkt.charge_status = is_device_charging() ? 1 : 0;

    // 5️⃣ Modbus slave ID
    // pkt.modbus_slave_id = get_modbus_slave_id();

    // 6️⃣ Compute CRC
   // pkt.crc = crc16((uint8_t *)&pkt, sizeof(pkt) - 2);
    return pkt;
}

void get_device_id(uint8_t *id) 
{
    esp_read_mac(id, ESP_MAC_BLE);
}

int8_t get_ble_rssi(void)
{
    return -55;
}

uint16_t get_battery_mv(void)
{
       static esp_adc_cal_characteristics_t adc_chars;
       static bool initialized = false;

       if (!initialized) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(ADC_BAT_CHANNEL, ADC_ATTEN);
         esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH_BIT_12, 1100, &adc_chars);
        initialized = true;
       }

       uint32_t adc_reading = 0;
       for (int i = 0; i < 10; i++) adc_reading += adc1_get_raw(ADC_BAT_CHANNEL);
        adc_reading /= 10;
        uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars);
        voltage_mv *= 11;  
        return (uint16_t)voltage_mv;
}

bool is_device_charging(void)
{
    static bool gpio_init = false;

    if (!gpio_init) {
     gpio_set_direction(CHARGING_GPIO, GPIO_MODE_INPUT);
     gpio_set_pull_mode(CHARGING_GPIO, GPIO_PULLDOWN_ONLY);
     gpio_init = true;
    }
    
    return (gpio_get_level(CHARGING_GPIO) == 1);
}


void soil_ble_task(void *pv)
{
    soil_packet_t soil_packet;

    while (1)
    {
        // Wait for notification to send data
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        build_soil_packet(&soil_packet);

        // Send packet via BLE
        uint16_t total_len = sizeof(soil_packet.data_type) +
                             sizeof(soil_packet.device_id) +
                             sizeof(soil_packet.soil8in1) +
                             sizeof(soil_packet.reserved) +
                             sizeof(soil_packet.crc);

        // 4️⃣ Send the packet over BLE
        sendOverBLE_soil_packet(soil_packet, total_len);
    }
}

void ping_ble_task(void *pv)
{
    while (1)
    {
        // Build ping packet
        ping_packet_t pkt = build_ping_packet();

        // Send over BLE
        sendOverBLE_ping_packet(pkt, sizeof(ping_packet_t));
        // Wait 2 seconds 
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}