#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "freertos/semphr.h"
#include "mqtt.h"
#include "mqttConfig.h"

#define MQTTPUBLISH 0x30
#define MQTTPUBACK 0x40
#define MQTTPINGREQ 0xC0
#define MQTTPINGRESP 0xD0
#define MQTTQOS1 0x02
#define MQTT_BUFFER_SIZE 512
#define TAG "MQTT_TASK"

static uint32_t last_in_activity = 0;
static uint32_t last_out_activity = 0;
static bool ping_outstanding = false;
networkState currentStatus;
static uint16_t mqtt_packet_id = 1;
TaskHandle_t publishHandleTask_handle = NULL;
TaskHandle_t connectionHandleTask_handle = NULL;
TaskFunction_t receiveHandleTask_handle = NULL;
extern TaskHandle_t dataLoggingTask_Handle;

void disconnectTCP(void)
{
    sendCmd_01("AT+CIPCLOSE", "CLOSE OK", 3000);
}
bool setupGPRS(const char *apn)
{
    uint8_t max_try = 3;
    while (send_command_02("AT+CREG?", "+CREG: 0,1", "+CREG: 0,2", 1000) == 0 && max_try > 0)
    {
        max_try--;
    }
    if (max_try == 0)
    {
        currentStatus = NETWORK_STATE_GPRS_ERROR;
        simHardHeset(10);
        return initGPRS(apn);
    }
    else
    {
        currentStatus = NETWORK_STATE_GPRS_ATTACHED;
        return true;
    }
}

void initGPIO(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MODEM_RESET_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};

    gpio_config(&io_conf);
    gpio_set_level(MODEM_RESET_PIN, 1); // Set high (inactive) by default
}

void simHardHeset(uint8_t wait_seconds)
{
    // Perform hard reset sequence
    gpio_set_level(MODEM_RESET_PIN, 0);
    currentStatus = NETWORK_STATE_OFF;
    vTaskDelay(pdMS_TO_TICKS(500)); // hold LOW for 500ms
    gpio_set_level(MODEM_RESET_PIN, 1);
    currentStatus = NETWORK_STATE_GPRS_ERROR;
    vTaskDelay(pdMS_TO_TICKS(wait_seconds * 1000)); // wait 10 seconds after reset
    // ESP_LOGI(TAG, "SIM800L reset done");
}

void initUART(void)
{
    uart_config_t config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(MODEM_UART_PORT, &config);
    uart_set_pin(MODEM_UART_PORT, MODEM_UART_TX, MODEM_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(MODEM_UART_PORT, 2048, 0, 0, NULL, 0);
}

void sendCmd_0(const char *cmd)
{

    uart_write_bytes(MODEM_UART_PORT, cmd, strlen(cmd));
    uart_write_bytes(MODEM_UART_PORT, "\r\n", 2);
}

uint8_t sendCmd_01(const char *ATcommand, const char *expected_answer, uint32_t timeout_ms)
{
    char response[256] = {0};
    int response_len = 0;
    uint8_t answer = 0;
    uint8_t byte = 0;
    int len = 0;

    // Clear UART RX buffer

    while (uart_read_bytes(MODEM_UART_PORT, &byte, 1, pdMS_TO_TICKS(10)) > 0)
        ;

    // Send the AT command

    uart_write_bytes(MODEM_UART_PORT, ATcommand, strlen(ATcommand));
    uart_write_bytes(MODEM_UART_PORT, "\r\n", 2);
    // ESP_LOGI(TAG, "Sent: %s", ATcommand);

    uint32_t start = esp_timer_get_time() / 1000;

    // Read incoming response
    while ((esp_timer_get_time() / 1000 - start) < timeout_ms && response_len < sizeof(response) - 1)
    {

        len = uart_read_bytes(MODEM_UART_PORT, &byte, 1, pdMS_TO_TICKS(10));
        if (len > 0)
        {
            response[response_len++] = byte;
            response[response_len] = '\0';

            if (strstr(response, expected_answer) != NULL)
            {
                answer = 1;
                break;
            }
        }
    }
    // ESP_LOGI(TAG, "Response: %s", response);
    return answer;
}
uint8_t send_command_02(const char *ATcommand, const char *expected1, const char *expected2, uint32_t timeout_ms)
{
    char response[256] = {0};
    int response_len = 0;
    uint8_t byte = 0;
    int len = 0;
    uint8_t result = 0;

    // Clear UART RX buffer
    while (uart_read_bytes(MODEM_UART_PORT, &byte, 1, pdMS_TO_TICKS(10)) > 0)
        ;

    // Send the AT command
    uart_write_bytes(MODEM_UART_PORT, ATcommand, strlen(ATcommand));
    uart_write_bytes(MODEM_UART_PORT, "\r\n", 2);
    // ESP_LOGI(TAG, "Sent: %s", ATcommand);

    int64_t start = esp_timer_get_time() / 1000;

    // Read incoming response
    while ((esp_timer_get_time() / 1000 - start) < timeout_ms && response_len < sizeof(response) - 1)
    {
        len = uart_read_bytes(MODEM_UART_PORT, &byte, 1, pdMS_TO_TICKS(10));
        if (len > 0)
        {
            response[response_len++] = byte;
            response[response_len] = '\0';

            if (expected1 && strstr(response, expected1))
            {
                result = 1;
                break;
            }
            else if (expected2 && strstr(response, expected2))
            {
                result = 2;
                break;
            }
        }
    }
    // ESP_LOGI(TAG, "Response: %s", response);
    return result;
}
int readResponse(char *resp_buf, size_t max_len, int timeout_ms)
{

    int len = 0;

    len = uart_read_bytes(MODEM_UART_PORT, (uint8_t *)resp_buf, max_len - 1, pdMS_TO_TICKS(timeout_ms));
    if (len > 0)
    {
        resp_buf[len] = 0;
        // ESP_LOGI(TAG, "Resp: %s", resp_buf);
    }
    return len;
}
bool initGPRS(const char *apn)
{
    char cmd[128];

    if (!sendCmd_01("ATE0", "OK", 500))
    {
        ESP_LOGE(TAG, "disableEcho command failed");
        return false;
    }
    if (!sendCmd_01("AT+GSN", "OK", 1000))
    {
        // ESP_LOGE(TAG, "IME command failed");
        return false;
    }

    if (!sendCmd_01("AT+CSMINS?", "+CSMINS: 0,1", 1000))
    {
        // ESP_LOGE(TAG, "IME command failed");
        return false;
    }
    currentStatus = NETWORK_STATE_SIM_READY;
    if (!send_command_02("AT+CREG?", "+CREG: 0,1", "+CREG: 0,2", 1000))
    {
        // ESP_LOGE(TAG, "AT+CREG? command failed");
        return false;
    }
    currentStatus = NETWORK_STATE_GPRS_ATTACHED;

    return true;
}

bool connectTCP(const char *host, int port)
{
    char cmd[128];

    // Step 1: Disable SSL
    if (!sendCmd_01("AT+CIPSSL=0", "OK", 5000))
    {
        ESP_LOGE("SIM800L", "Failed to disable SSL");
        return false;
    }
    // Step 2: Close any existing TCP connection
    if (!sendCmd_01("AT+CIPSHUT", "SHUT OK", 3000))
    {
        ESP_LOGW("SIM800L", "CIPSHUT failed, retrying...");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait a bit
        if (!sendCmd_01("AT+CIPSHUT", "SHUT OK", 3000))
        {
            ESP_LOGE("SIM800L", "CIPSHUT failed after retry");
            return false;
        }
    }
    // Step 3: Optional - check current IP status (not mandatory but useful)
    sendCmd_01("AT+CIPSTATUS", "STATE", 1000); // ✅ safely check for a prefix

    // Step 4: Set to single connection mode
    if (!sendCmd_01("AT+CIPMUX=0", "OK", 1000))
    {
        ESP_LOGE("SIM800L", "Failed to set CIPMUX");
        return false;
    }

    // Step 5: Enable manual read mode for TCP
    if (!sendCmd_01("AT+CIPRXGET=1", "OK", 1000))
    {
        ESP_LOGE("SIM800L", "Failed to enable manual TCP read");
        return false;
    }

    // Step 6: Start the TCP connection
    // snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",\"%d\"", host, port);
    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",\"%d\"", host, port);
    if (!sendCmd_01(cmd, "CONNECT OK", 8000))
    {
        ESP_LOGE("SIM800L", "TCP Connect failed");
        return false;
    }

    ESP_LOGI("SIM800L", "TCP connection established successfully");
    return true;
}

bool isConnected()
{
    if (!sendCmd_01("AT+CIPSTATUS", "OK", 1000))
        return false;
    if (!sendCmd_01("", "STATE: CONNECT OK", 2000))
        return false;
    return true;
}
bool sendTCP(const uint8_t *data, int len)
{
    char cmd[32];
    int attempts = 0;

    while (attempts < 5)
    {
        attempts++;

        sprintf(cmd, "AT+CIPSEND=%d", len);

        if (!sendCmd_01(cmd, ">", 2000))
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue; // Try again
        }

        uart_write_bytes(MODEM_UART_PORT, (const char *)data, len);

        if (!sendCmd_01("", "SEND OK", 5000))
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue; // Try again
        }
        // Success
        return true;
    }
    ESP_LOGE(TAG, "sendTCP failed after 5 attempts");
    xTaskNotifyGive(connectionHandleTask_handle);
    return false;
}

int readTcpPayload(uint8_t *data_buf, uint32_t timeout_ms)
{
    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    while ((xTaskGetTickCount() * portTICK_PERIOD_MS - start_time) < timeout_ms)
    {

        // Step 1: Check available TCP bytes
        sendCmd_0("AT+CIPRXGET=4");
        char resp_buf[128];
        int resp_len = readResponse(resp_buf, sizeof(resp_buf), 500);

        char *p = strstr(resp_buf, "+CIPRXGET: 4,");
        if (!p)
        {
            continue;
        }

        int available_bytes = atoi(p + strlen("+CIPRXGET: 4,"));
        if (available_bytes == 0)
        {
            continue;
        }

        if (available_bytes > MQTT_BUFFER_SIZE)
        {
            available_bytes = MQTT_BUFFER_SIZE;
        }

        // Step 2: Read TCP data
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "AT+CIPRXGET=2,%d", available_bytes);
        sendCmd_0(cmd);

        char read_buf[512];
        int read_len = readResponse(read_buf, sizeof(read_buf), 2000);
        if (read_len <= 0)
        {
            continue;
        }
        // Step 3: Find binary payload
        p = strstr(read_buf, "+CIPRXGET: 2,");
        if (!p)
            continue;

        p = strstr(p, "\r\n");
        if (!p)
            continue;

        p += 2; // Skip \r\n to reach binary

        memcpy(data_buf, p, available_bytes);
        return available_bytes;
    }

    return 0; // No data or timeout
}

bool mqttConnect(const char *client_id, const char *username, const char *password)
{
    uint8_t packet[256];
    uint8_t payload[192];
    size_t i = 0, p = 0;

    // --- Fixed header ---
    packet[i++] = 0x10; // CONNECT packet type

    // --- Variable header ---
    payload[p++] = 0x00;
    payload[p++] = 0x04;
    payload[p++] = 'M';
    payload[p++] = 'Q';
    payload[p++] = 'T';
    payload[p++] = 'T';
    payload[p++] = 0x04; // Protocol Level (MQTT 3.1.1)

    uint8_t connect_flags = 0x02; // Clean session
    if (username)
        connect_flags |= 0x80;
    if (password)
        connect_flags |= 0x40;
    payload[p++] = connect_flags;

    payload[p++] = 0x00;
    payload[p++] = 15; // Keep Alive = 15 sec

    // --- Payload: Client ID ---
    uint16_t id_len = strlen(client_id);
    payload[p++] = id_len >> 8;
    payload[p++] = id_len & 0xFF;
    memcpy(&payload[p], client_id, id_len);
    p += id_len;

    // --- Payload: Username ---
    if (username)
    {
        uint16_t user_len = strlen(username);
        payload[p++] = user_len >> 8;
        payload[p++] = user_len & 0xFF;
        memcpy(&payload[p], username, user_len);
        p += user_len;
    }
    // --- Payload: Password ---
    if (password)
    {
        uint16_t pass_len = strlen(password);
        payload[p++] = pass_len >> 8;
        payload[p++] = pass_len & 0xFF;
        memcpy(&payload[p], password, pass_len);
        p += pass_len;
    }
    // --- Remaining Length Encoding ---
    size_t remaining_len = p;
    do
    {
        uint8_t byte = remaining_len % 128;
        remaining_len /= 128;
        if (remaining_len > 0)
            byte |= 0x80;
        packet[i++] = byte;
    } while (remaining_len > 0);

    // --- Finalize packet ---
    memcpy(&packet[i], payload, p);
    size_t total_len = i + p;

    // --- Send MQTT CONNECT packet over TCP ---
    if (!sendTCP(packet, total_len))
    {
        // ESP_LOGE("MQTT", "Failed to send CONNECT packet");
        return false;
    }

    uart_flush_input(MODEM_UART_PORT); // Clean any junk
    // --- Wait for CONNACK ---
    uint8_t connack_buf[8];
    int read_len = readTcpPayload(connack_buf, 5000); // 5 sec timeout

    if (read_len >= 4)
    {
        // ESP_LOGI("MQTT", "Received %d bytes", read_len);
        // ESP_LOG_BUFFER_HEX("MQTT_CONNACK", connack_buf, read_len);

        if (connack_buf[0] == 0x20 && connack_buf[1] == 0x02)
        {
            if (connack_buf[3] == 0x00)
            {
                // ESP_LOGI("MQTT", "MQTT CONNECT Success (CONNACK OK)");
                return true;
            }
            else
            {
                // ESP_LOGE("MQTT", "MQTT CONNECT Failed, return code: 0x%02X", connack_buf[3]);
                return false;
            }
        }
        else
        {
            // ESP_LOGE("MQTT", "Invalid CONNACK format");
        }
    }
    else
    {
        // ESP_LOGE("MQTT", "No CONNACK or short packet");
    }

    return false;
}
bool connectBroker(const char *broker_address, int broker_port, const char *client_id, const char *username, const char *password)
{
    // Step 1: Establish TCP connection
    if (!connectTCP(broker_address, broker_port))
    {
        ESP_LOGE("MQTT", "Failed to connect to TCP broker");
        return false;
    }
    currentStatus = MQTT_STATE_TCP_CONNECTED;
    // Step 2: Send MQTT CONNECT packet
    if (!mqttConnect(client_id, username, password))
    {
        ESP_LOGE("MQTT", "Failed to connect to MQTT broker");
        return false;
    }
    currentStatus = MQTT_STATE_MQTT_CONNECTED;

    ESP_LOGI("MQTT", "Successfully connected to MQTT broker");
    return true;
}

bool mqttSubscribe(uint16_t packet_id, const char *topic)
{
    uint16_t topic_len = strlen(topic);
    uint16_t remaining_len = 2 + 2 + topic_len + 1; // packet_id + topic_len(2) + topic + QoS
    uint8_t buffer[128];
    int i = 0;
    // Fixed header
    buffer[i++] = 0x82; // SUBSCRIBE packet type + flags
    // MQTT remaining length can use variable length encoding
    buffer[i++] = remaining_len;
    // Variable header (packet ID)
    buffer[i++] = (packet_id >> 8) & 0xFF;
    buffer[i++] = packet_id & 0xFF;

    // Payload: topic + QoS
    buffer[i++] = (topic_len >> 8) & 0xFF;
    buffer[i++] = topic_len & 0xFF;

    memcpy(&buffer[i], topic, topic_len);
    i += topic_len;

    buffer[i++] = 0x00; // QoS = 0

    if (!sendTCP(buffer, i))
    {
        ESP_LOGE(TAG, "MQTT SUBSCRIBE failed");
        disconnectTCP();
        return false;
    }
    return true;
}
// Encodes a string with 2-byte length prefix
static uint8_t *encodeString(uint8_t *buf, const char *str)
{
    uint16_t len = strlen(str);
    *buf++ = len >> 8;
    *buf++ = len & 0xFF;
    memcpy(buf, str, len);
    return buf + len;
}

int mqttPublishPack(uint8_t *packet, const char *topic, const uint8_t *data, uint16_t data_len, uint8_t qos)
{
    uint8_t *p = packet;
    uint16_t len = 0;

    // === Fixed header ===
    *p++ = 0x30 | (qos << 1); // PUBLISH (QoS encoded in bit 1)

    // === Remaining length (calculated below) ===
    uint16_t remaining_len = 2 + strlen(topic); // Topic length + string
    if (qos > 0)
        remaining_len += 2;    // Packet ID
    remaining_len += data_len; // Payload

    // Variable-length encoding of remaining length
    do
    {
        uint8_t encoded = remaining_len % 128;
        remaining_len /= 128;
        if (remaining_len > 0)
            encoded |= 0x80;
        *p++ = encoded;
    } while (remaining_len > 0);

    // === Variable header ===
    p = encodeString(p, topic); // Topic

    if (qos > 0)
    {
        *p++ = mqtt_packet_id >> 8;
        *p++ = mqtt_packet_id & 0xFF;
        mqtt_packet_id++;
    }

    // === Payload ===
    memcpy(p, data, data_len);
    p += data_len;

    return p - packet; // Return total packet length
}

void mqttCallback(char *topic, uint8_t *payload, unsigned int len)
{
    ESP_LOGI("MQTT_CB", "Topic: %s", topic);

    // Ensure payload is printable (null-terminated)
    char msg[256] = {0}; // adjust size based on expected payload
    if (len >= sizeof(msg))
        len = sizeof(msg) - 1; // prevent overflow
    memcpy(msg, payload, len);
    msg[len] = '\0';

    ESP_LOGI("MQTT_CB", "Payload: %s", msg);

    // You can now add logic to parse or handle this payload
    // Example: If topic matches specific command
}

bool mqttCheckSubscription(void (*mqtt_callback)(char *topic, uint8_t *payload, unsigned int len))
{

    uint32_t now = esp_log_timestamp();

    // --- Keep-Alive Ping ---
    if ((now - last_in_activity > MQTT_KEEPALIVE * 1000) ||
        (now - last_out_activity > MQTT_KEEPALIVE * 1000))
    {

        // If ping_outstanding and timeout exceeded 5 seconds after keepalive
        if (ping_outstanding &&
            (now - last_out_activity > (MQTT_KEEPALIVE + 5) * 1000))
        {
            ping_outstanding = false;
            last_in_activity = last_out_activity = now;
            ESP_LOGE("MQTT_LOOP", "Ping timeout (no PINGRESP within 5 sec)");
            xTaskNotifyGive(connectionHandleTask_handle);
            return false; // Consider disconnected
        }

        if (!ping_outstanding)
        {
            // Send PINGREQ
            uint8_t ping_req[] = {MQTTPINGREQ, 0x00};
            if (!sendTCP(ping_req, 2))
            {
                ESP_LOGE("MQTT_LOOP", "Failed to send PINGREQ");
                return false;
            }
            ESP_LOGI("MQTT_LOOP", "Sent PINGREQ");
            last_out_activity = now;
            ping_outstanding = true;
        }
    }
    // --- Read MQTT Packet ---
    uint8_t mqtt_rx_buffer[512] = {0};
    int read_len = readTcpPayload(mqtt_rx_buffer, 1000);
    if (read_len <= 0)
    {
        return true; // no data yet
    }

    uint8_t packet_type = mqtt_rx_buffer[0] & 0xF0;

    switch (packet_type)
    {
    case MQTTPINGRESP:
        ESP_LOGI("MQTT_LOOP", "Received PINGRESP");
        ping_outstanding = false;
        last_in_activity = now;
        break;

    case MQTTPUBLISH:
    {
        ping_outstanding = false;
        last_in_activity = now;
        uint8_t *data = mqtt_rx_buffer;
        uint32_t len_val = 0;
        uint32_t multiplier = 1;
        uint8_t i = 1;
        uint8_t rem_len;

        do
        {
            rem_len = data[i];
            len_val += (rem_len & 127) * multiplier;
            multiplier *= 128;
        } while ((data[i++] & 128) != 0 && i < read_len);

        uint16_t topic_len = (data[i] << 8) | data[i + 1];
        char topic[128] = {0};
        memcpy(topic, &data[i + 2], topic_len);
        topic[topic_len] = '\0';

        uint8_t *payload = &data[i + 2 + topic_len];
        uint16_t payload_len = len_val - topic_len - 2;

        if (mqtt_callback)
        {
            mqtt_callback(topic, payload, payload_len);
        }

        // QoS 1 PUBACK
        if ((data[0] & 0x06) == MQTTQOS1)
        {
            uint16_t msg_id = (payload[0] << 8) | payload[1];
            uint8_t puback[] = {MQTTPUBACK, 0x02, msg_id >> 8, msg_id & 0xFF};
            sendTCP(puback, 4);
            ESP_LOGI("MQTT_LOOP", "Sent PUBACK for msg_id=%d", msg_id);
        }

        last_in_activity = now;
        break;
    }
    default:
        // ESP_LOGW("MQTT_LOOP", "Unhandled MQTT packet type: 0x%02X", packet_type);
        // last_in_activity = now;
        break;
    }
    return true;
}

static void connectionHandleTask(void *pvParameters)
{
    uint8_t connect_buf[256];

    while (1)
    {
        ESP_LOGI(TAG, "Resetting and reinitializing SIM800L...");

        if (!setupGPRS(MQTT_APN))
            continue;

        if (!connectBroker(MQTT_HOST, MQTT_PORT, USER_ID, USER_NAME, USER_PASSWORD))
        {
            ESP_LOGE(TAG, "MQTT CONNECT failed");
            disconnectTCP();
            continue;
        }

        if (!mqttSubscribe(10, TOPIC))
            continue;

        currentStatus = MQTT_STATE_SUBSCRIBE;
        xTaskNotifyGive(receiveHandleTask_handle); // Notify the receive task to start listening
        ESP_LOGI(TAG, "MQTT Connected and Subscribed");
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for disconnect notice
    }
}

static void publishHandleTask(void *pvParameters)
{
    M_payload_t received_payload;
    uint8_t mqtt_packet[512];

    while (1)
    {

        if (xQueueReceive(modbus_payload_queue, &received_payload, portMAX_DELAY) == pdPASS)
        {

            // Debug print the payload content
            ESP_LOGI("MQTT", "Payload received:");
            ESP_LOGI("MQTT", "  Timestamp: %llu", received_payload.timestamp);
            ESP_LOG_BUFFER_HEX("MQTT", received_payload.device_id, sizeof(received_payload.device_id));
            ESP_LOGI("MQTT", "  Topic: %s", received_payload.topic);
            ESP_LOGI("MQTT", "  Battery (mV): %u", received_payload.battery_mv);

            for (int i = 0; i < 20; i++)
            {
                ESP_LOGI("MQTT", "  Sensor %02d: pH=%u, Moisture=%u, Temp=%d, EC=%u, N=%u, P=%u, K=%u",
                         i,
                         received_payload.sensor_data[i].ph,
                         received_payload.sensor_data[i].moisture,
                         received_payload.sensor_data[i].temperature,
                         received_payload.sensor_data[i].conductivity,
                         received_payload.sensor_data[i].nitrogen,
                         received_payload.sensor_data[i].phosphorus,
                         received_payload.sensor_data[i].potassium);
            }

            ESP_LOG_BUFFER_HEX("MQTT", received_payload.reserved, sizeof(received_payload.reserved));
            ESP_LOGI("MQTT", "  CRC: 0x%04X", received_payload.crc);

            int len = mqttPublishPack(
                mqtt_packet,
                (const char *)received_payload.topic,
                (const uint8_t *)&received_payload,
                sizeof(received_payload),
                0);

            if (currentStatus >= MQTT_STATE_MQTT_CONNECTED)
            {

                if (!sendTCP(mqtt_packet, len))
                {
                    ESP_LOGE("MQTT_PUB", "Failed to publish, notifying reconnect");
                    // QUEUE TO THE NVS HERE-----------------------------
                    ESP_LOGE("MQTT_PUB", "Failed to publish, queue to nvs");

                    //----------------------------------------------------
                    currentStatus = MQTT_STATE_DISCONNECTED;
                    xTaskNotifyGive(connectionHandleTask_handle); // Trigger reconnect
                }
                else
                    ESP_LOGW("MQTT_PUB", "Published sensor data");
            }
            else
            {
                // QUEUE TO THE NVS HERE-----------------------------
                ESP_LOGE("MQTT_PUB", "Failed to publish, queue to nvs");
                //----------------------------------------------------
            }
        }
    }
}

void mqtt_callback(char *topic, uint8_t *payload, unsigned int len)
{
    // Null-terminate the payload
    char message[256];
    if (len >= sizeof(message))
        len = sizeof(message) - 1;

    memcpy(message, payload, len);
    message[len] = '\0';
    ESP_LOGI("MQTT_CALLBACK", "Received on topic: %s", topic);
    ESP_LOGI("MQTT_CALLBACK", "Message: %s", message);

    // parse, act on topic, send response, etc.
}

void receiveHandleTask(void *pvParameters)
{
    while (1)
    {
        // Wait until SIM800L is connected and subscribed
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (currentStatus != MQTT_STATE_SUBSCRIBE)
        {
            ESP_LOGW("MQTT_RECV", "SIM800L not yet subscribed. Waiting...");
            continue;
        }

        ESP_LOGI("MQTT_RECV", "Starting MQTT receive loop");

        while (currentStatus == MQTT_STATE_SUBSCRIBE || currentStatus == MQTT_STATE_MQTT_CONNECTED)
        {
            if (!mqttCheckSubscription(mqtt_callback))
                break;
        }
        ESP_LOGW("MQTT_RECV", "Disconnected. Waiting for reconnect...");
    }
}

void initTask(void)
{
    xTaskCreatePinnedToCore(connectionHandleTask, "connectionHandleTask", 1024 * 4, NULL, 2, &connectionHandleTask_handle, 0);
    xTaskCreatePinnedToCore(receiveHandleTask, "receiveHandleTask", 1024 * 6, NULL, 5, &receiveHandleTask_handle, 1);
    xTaskCreatePinnedToCore(publishHandleTask, "publishHandleTask", 1024 * 4, NULL, 2, &publishHandleTask_handle, 0);
    xTaskCreatePinnedToCore(dataLoggingHandleTask, "data_collect_task", 1024 * 4, NULL, 2, &dataLoggingTask_Handle, 0);
}