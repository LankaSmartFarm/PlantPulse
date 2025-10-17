
#include <string.h>
#include "dataLogging.h"
#include "BLE/ble50_sec_gatts.h"

uint8_t modbusSlaveAddress=0x01; // Default Modbus slave address
esp_adc_cal_characteristics_t *adc_chars_bat = NULL;

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
        MB_rtu_send(modbusSlaveAddress, SOIL8IN1_FUNC_CODE, SOIL8IN1_REG_ADDR, SOIL8IN1_REG_COUNT, tx_buf, 0);

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
// void build_soil_packet(soil_packet_t *packet)
// {
//     // 1️⃣ Fill entire struct with 0xFF initially
//     memset(packet, 0xFF, sizeof(*packet));

//     // 2️⃣ Data type for identification
//     packet->data_type = SOIL_PACKET_TYPE;

//     // 3️⃣ Device ID (6 bytes)
//     get_device_id(packet->device_id);

//     // 4️⃣ Read soil sensor data
//     packet->soil8in1 = build_soil8in1_data();

//     // 5️⃣ Compute CRC (excluding last 2 bytes)
//     packet->crc = crc16((uint8_t *)packet, sizeof(*packet) - sizeof(packet->crc));

//     // 6️⃣ Print formatted packet fields
//     printf("\n================ SOIL PACKET =================\n");
//     printf("Data Type        : 0x%08lX\n", packet->data_type);
//     printf("Device ID        : ");
//     for (int i = 0; i < 6; i++) printf("%02X ", packet->device_id[i]);
//     printf("\n");

//     printf("Temperature      : %u (%.2f°C)\n", packet->soil8in1.temperature, packet->soil8in1.temperature / 100.0);
//     printf("Humidity         : %u (%.2f%%)\n", packet->soil8in1.humidity, packet->soil8in1.humidity / 100.0);
//     printf("EC               : %u µS/cm\n", packet->soil8in1.ec);
//     printf("pH               : %u (%.2f)\n", packet->soil8in1.ph, packet->soil8in1.ph / 100.0);
//     printf("Nitrogen         : %u mg/kg\n", packet->soil8in1.nitrogen);
//     printf("Phosphorus       : %u mg/kg\n", packet->soil8in1.phosphorus);
//     printf("Potassium        : %u mg/kg\n", packet->soil8in1.potassium);
//     printf("Soil Moisture    : %u (%.2f%%)\n", packet->soil8in1.soil_moisture, packet->soil8in1.soil_moisture / 100.0);

//     printf("CRC              : 0x%04X\n", packet->crc);
//     printf("================================================\n\n");

//     // 7️⃣ Hex dump for full packet
//     ESP_LOG_BUFFER_HEX("SOIL_PACKET", packet, sizeof(*packet));
// }
void build_soil_packet(soil_packet_t *packet)
{
    // 1️⃣ Fill entire struct with 0xFF initially
    memset(packet, 0xFF, sizeof(*packet));

    // 2️⃣ Data type (4 bytes) → big-endian
    packet->data_type = __builtin_bswap32(SOIL_PACKET_TYPE);

    // 3️⃣ Device ID (6 bytes)
    get_device_id(packet->device_id);

    // 4️⃣ Read soil sensor data
    soil8in1_data_t soil = build_soil8in1_data();

    // 5️⃣ Assign sensor values in big-endian
    packet->soil8in1.temperature   = __builtin_bswap16(soil.temperature);
    packet->soil8in1.humidity      = __builtin_bswap16(soil.humidity);
    packet->soil8in1.ec            = __builtin_bswap16(soil.ec);
    packet->soil8in1.ph            = __builtin_bswap16(soil.ph);
    packet->soil8in1.nitrogen      = __builtin_bswap16(soil.nitrogen);
    packet->soil8in1.phosphorus    = __builtin_bswap16(soil.phosphorus);
    packet->soil8in1.potassium     = __builtin_bswap16(soil.potassium);
    packet->soil8in1.soil_moisture = __builtin_bswap16(soil.soil_moisture);

    // 6️⃣ Compute CRC (excluding last 2 bytes) → then convert to big-endian
    uint16_t crc = crc16((uint8_t *)packet, sizeof(*packet) - sizeof(packet->crc));
    packet->crc = __builtin_bswap16(crc);

    // 7️⃣ Full hex dump
   // ESP_LOG_BUFFER_HEX("SOIL_PACKET", packet, sizeof(*packet));

    // 8️⃣ Print all fields
    printf("\n================ SOIL PACKET =================\n");
    printf("Data Type        : 0x%08X\n", SOIL_PACKET_TYPE);
    
    printf("Device ID        : ");
    for (int i = 0; i < 6; i++) printf("%02X ", packet->device_id[i]);
    printf("\n");

    // Sensor fields (show hex + decimal/converted)
    printf("Temperature      : 0x%04X / %.2f °C\n", soil.temperature, soil.temperature / 100.0);
    printf("Humidity         : 0x%04X / %.2f %%\n", soil.humidity, soil.humidity / 100.0);
    printf("EC               : 0x%04X / %u µS/cm\n", soil.ec, soil.ec);
    printf("pH               : 0x%04X / %.2f\n", soil.ph, soil.ph / 100.0);
    printf("Nitrogen         : 0x%04X / %u mg/kg\n", soil.nitrogen, soil.nitrogen);
    printf("Phosphorus       : 0x%04X / %u mg/kg\n", soil.phosphorus, soil.phosphorus);
    printf("Potassium        : 0x%04X / %u mg/kg\n", soil.potassium, soil.potassium);
    printf("Soil Moisture    : 0x%04X / %.2f %%\n", soil.soil_moisture, soil.soil_moisture / 100.0);

    printf("CRC              : 0x%04X\n", crc);
    printf("================================================\n\n");
}



static void charge_init(void)
{

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CHARGE_DETECT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << BATTERY_CHARGE_STATE);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    // Configure ADC for PIR sensor (ADC1)
    adc1_config_width(ADC_WIDTH_BIT_12);                             // 12-bit resolution
    adc1_config_channel_atten(BATTERY_ADC_CHANNEL, ADC_ATTEN_DB_11); // 0-3.6V range
    adc_chars_bat = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars_bat);
}

static uint8_t is_device_charging(void)
{
    // Assuming LOW = charging
    return gpio_get_level(CHARGE_DETECT_GPIO);
}
static uint16_t get_battery_mv(void)
{

    if (adc_chars_bat != NULL)
    {

        uint32_t adc_reading = 0;
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            adc_reading += adc1_get_raw(BATTERY_ADC_CHANNEL);
        }
        adc_reading /= NO_OF_SAMPLES;

        uint16_t bat_adc = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars_bat); // Return voltage in mV
        printf("bat_adc : %d mV\n", bat_adc);
        return bat_adc; 
    }
    return 0xFFFF;
}

ping_packet_t build_ping_packet(void)
{
    ping_packet_t pkt;
    memset(&pkt, 0xFF, sizeof(pkt)); // Fill all with 0xFF initially

    // 1️⃣ Packet type (Big Endian)
    pkt.packet_type = __builtin_bswap32(PING_PACKET_TYPE);

    // 2️⃣ Device ID (6 bytes)
    get_device_id(pkt.device_id);

    // 3️⃣ Runtime (seconds)
    uint32_t runtime = xTaskGetTickCount() / 1000;
    pkt.runTime = __builtin_bswap32(runtime);

    // 4️⃣ Battery level (millivolts)
    uint16_t batt_mv = get_battery_mv();
    pkt.battery_level = __builtin_bswap16(batt_mv);

    // 5️⃣ Charge indicator
    pkt.charge_status = is_device_charging() ? 0 : 1;

    // 6️⃣ Modbus slave ID
    pkt.modbus_slave_id = modbusSlaveAddress;

    // 7️⃣ Compute CRC (calculated on big-endian formatted data)
    uint16_t crc = crc16((uint8_t *)&pkt, sizeof(pkt) - 2);
    pkt.crc = __builtin_bswap16(crc);




    // 🧾 Debug Print Section
    printf("\n==== PING PACKET DEBUG ====\n");
    printf("Packet Type       : 0x%08X\n", PING_PACKET_TYPE);
    printf("Device ID         : ");
    for (int i = 0; i < 6; i++) printf("%02X", pkt.device_id[i]);
    printf("\n");
    printf("Runtime (dec)     : %ld sec\n", runtime);
    printf("Runtime (hex)     : 0x%08lX\n", runtime);
    printf("Battery (dec)     : %d mV\n", batt_mv);
    printf("Battery (hex)     : 0x%04X\n", batt_mv);
    printf("Charge Status     : %s\n", pkt.charge_status ? "Charging" : "Not Charging");
    printf("Modbus Slave ID   : %u\n", pkt.modbus_slave_id);
    printf("CRC (hex)         : 0x%04X\n", crc);
    printf("----------------------------\n");


    printf("Full Packet (HEX):\n");
    uint8_t *raw = (uint8_t *)&pkt;
    for (int i = 0; i < sizeof(pkt); i++) {
        printf("%02X ", raw[i]);
        if ((i + 1) % 16 == 0) printf("\n");
    }
    printf("\n============================\n");

    return pkt;
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

    charge_init();

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // Build ping packet
        ping_packet_t pkt = build_ping_packet();
        // Send over BLE
        sendOverBLE_ping_packet(pkt, sizeof(ping_packet_t));
    }
}