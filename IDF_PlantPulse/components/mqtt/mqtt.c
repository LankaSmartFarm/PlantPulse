#include <stdio.h>
#include "mqtt.h"
#include "mqttConfig.h"
#include <string.h>
#include <stdint.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "freertos/semphr.h"


static const char *TAG = "MQTT_TASK";
const char *myTopic = "test/topic";
const char *msg = "Hello MQTT";


static uint16_t mqtt_packet_id = 1;  // Global packet ID counter
SemaphoreHandle_t uart_mutex;

// #define TAG "SIM800L" 
extern SemaphoreHandle_t uart_mutex;

int mqtt_connect_packet(uint8_t *buf, const char *client_id, const char *username, const char *password) {
    int i = 0;
    buf[i++] = 0x10;
    buf[i++] = 0;

    buf[i++] = 0x00; buf[i++] = 0x04;
    memcpy(&buf[i], "MQTT", 4); i += 4;
    buf[i++] = 0x04;
    buf[i++] = 0xC2;
    buf[i++] = 0x00; buf[i++] = MQTT_KEEPALIVE;

    uint16_t len = strlen(client_id);
    buf[i++] = len >> 8; buf[i++] = len & 0xFF;
    memcpy(&buf[i], client_id, len); i += len;

    len = strlen(username);
    buf[i++] = len >> 8; buf[i++] = len & 0xFF;
    memcpy(&buf[i], username, len); i += len;

    len = strlen(password);
    buf[i++] = len >> 8; buf[i++] = len & 0xFF;
    memcpy(&buf[i], password, len); i += len;

    buf[1] = i - 2;
    return i;
}

int mqtt_subscribe_packet(uint8_t *buf, uint16_t packet_id, const char *topic)
{
    int i = 0;
    uint16_t topic_len = strlen(topic);

    buf[i++] = 0x82;  // MQTT SUBSCRIBE packet (type + flags)
    buf[i++] = 5 + topic_len;  // Remaining length

    // Packet Identifier
    buf[i++] = packet_id >> 8;
    buf[i++] = packet_id & 0xFF;

    // Topic
    buf[i++] = topic_len >> 8;
    buf[i++] = topic_len & 0xFF;
    memcpy(&buf[i], topic, topic_len); i += topic_len;

    // QoS
    buf[i++] = 0x00;

    return i;
}

bool  publish_sensor_data(float temp, float moisture, float pH, uint16_t cond,
                         uint16_t N, uint16_t P, uint16_t K,
                         uint16_t salinity, uint16_t tds)
{
    char payload[256];
    uint8_t packet[512];

    snprintf(payload, sizeof(payload),
             "{\"temperature\":%.1f,\"moisture\":%.1f,\"pH\":%.1f,"
             "\"conductivity\":%u,\"nitrogen\":%u,\"phosphorus\":%u,"
             "\"potassium\":%u,\"salinity\":%u,\"tds\":%u}",
             temp, moisture, pH, cond, N, P, K, salinity, tds);

    ESP_LOGI("MQTT", "Publishing: %s", payload);

    int len = mqtt_publish_packet(packet, TOPIC, (const uint8_t *)payload, strlen(payload), 0);

    if (!sim800l_send_tcp(packet, len)) {
        ESP_LOGE("MQTT", "Failed to publish sensor data");
        return false;
    } else {
        ESP_LOGI("MQTT", "Sensor data published successfully");
        return true;
    }
}



// Encodes a string with 2-byte length prefix
static uint8_t *mqtt_encode_string(uint8_t *buf, const char *str) {
    uint16_t len = strlen(str);
    *buf++ = len >> 8;
    *buf++ = len & 0xFF;
    memcpy(buf, str, len);
    return buf + len;
}

int mqtt_publish_packet(uint8_t *packet, const char *topic, const uint8_t *data, uint16_t data_len, uint8_t qos)
{
    uint8_t *p = packet;
    uint16_t len = 0;

    // === Fixed header ===
    *p++ = 0x30 | (qos << 1);  // PUBLISH (QoS encoded in bit 1)

    // === Remaining length (calculated below) ===
    uint16_t remaining_len = 2 + strlen(topic); // Topic length + string
    if (qos > 0) remaining_len += 2;            // Packet ID
    remaining_len += data_len;                 // Payload

    // Variable-length encoding of remaining length
    do {
        uint8_t encoded = remaining_len % 128;
        remaining_len /= 128;
        if (remaining_len > 0) encoded |= 0x80;
        *p++ = encoded;
    } while (remaining_len > 0);

    // === Variable header ===
    p = mqtt_encode_string(p, topic);  // Topic

    if (qos > 0) {
        *p++ = mqtt_packet_id >> 8;
        *p++ = mqtt_packet_id & 0xFF;
        mqtt_packet_id++;
    }

    // === Payload ===
    memcpy(p, data, data_len);
    p += data_len;

    return p - packet;  // Return total packet length
}




void mqtt_handle_incoming(const uint8_t *data, int len) {
    if ((data[0] & 0xF0) == 0x30) {  // PUBLISH packet
        uint16_t topic_len = (data[2] << 8) | data[3];
        char topic[128] = {0};
        memcpy(topic, &data[4], topic_len);

        int payload_offset = 4 + topic_len;
        if ((data[0] & 0x06) != 0) payload_offset += 2; // QoS > 0 => skip packet ID

        char payload[256] = {0};
        int payload_len = len - payload_offset;
        memcpy(payload, &data[payload_offset], payload_len);

        ESP_LOGW("MQTT", "Received message on topic: %s", topic);
        ESP_LOGW("MQTT", "Payload: %s", payload);

        // Optionally, parse JSON or handle command
    }
}


void sim800l_disconnect_tcp(void) {
    sim800l_send_command("AT+CIPCLOSE", "CLOSE OK", 3000);
}
bool  sim800l_full_init(const char *apn) {
       
    uint8_t max_try= 3;
    while ( send_command("AT+CREG?", "+CREG: 0,1" ,"+CREG: 0,2", 1000) == 0 &&  max_try>0) {
        max_try--;
    }
    if (max_try==0) {
        currentStatus = NETWORK_STATE_GPRS_ERROR;
        sim800l_hard_reset(10);
        return sim800l_init_gprs(apn);
    }else {
        currentStatus = NETWORK_STATE_GPRS_ATTACHED;
        return true;
    }
}

void sim800l_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MODEM_RESET_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&io_conf);
    gpio_set_level(MODEM_RESET_PIN, 1);  // Set high (inactive) by default
}

void sim800l_hard_reset(uint8_t wait_seconds)
{

    // Perform hard reset sequence
    gpio_set_level(MODEM_RESET_PIN, 0);
    currentStatus = NETWORK_STATE_OFF;
    vTaskDelay(pdMS_TO_TICKS(500));   // hold LOW for 500ms
    gpio_set_level(MODEM_RESET_PIN, 1);
    currentStatus = NETWORK_STATE_GPRS_ERROR;
    vTaskDelay(pdMS_TO_TICKS(wait_seconds*1000)); // wait 10 seconds after reset
    // ESP_LOGI(TAG, "SIM800L reset done");

}



void sim800l_uart_init(void) {
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


void sim800l_send_cmd(const char *cmd) {

    uart_write_bytes(MODEM_UART_PORT, cmd, strlen(cmd));
    uart_write_bytes(MODEM_UART_PORT, "\r\n", 2);

}


int sim800l_wait_response(char *resp_buf, size_t max_len, int timeout_ms) {

    int len = 0;

    len = uart_read_bytes(MODEM_UART_PORT, (uint8_t*)resp_buf, max_len - 1, pdMS_TO_TICKS(timeout_ms));
    if (len > 0) {
        resp_buf[len] = 0;
        ESP_LOGI(TAG, "Resp: %s", resp_buf);
    }
return len;

}

uint8_t sim800l_send_command(const char *ATcommand, const char *expected_answer, uint32_t timeout_ms)
{
    char response[256] = {0};
    int response_len = 0;
    uint8_t answer = 0;
    uint8_t byte = 0;
    int len = 0;

    // Clear UART RX buffer

    while (uart_read_bytes(MODEM_UART_PORT, &byte, 1, pdMS_TO_TICKS(10)) > 0);


    // Send the AT command


    uart_write_bytes(MODEM_UART_PORT, ATcommand, strlen(ATcommand));
    uart_write_bytes(MODEM_UART_PORT, "\r\n", 2);
    // ESP_LOGI(TAG, "Sent: %s", ATcommand);


    uint32_t start = esp_timer_get_time() / 1000;



    // Read incoming response
    while ((esp_timer_get_time() / 1000 - start) < timeout_ms && response_len < sizeof(response) - 1) {

            len = uart_read_bytes(MODEM_UART_PORT, &byte, 1, pdMS_TO_TICKS(10));
            if (len > 0) {
                response[response_len++] = byte;
                response[response_len] = '\0';

                if (strstr(response, expected_answer) != NULL) {
                    answer = 1;
                    break;
                }
            }

    }
    // ESP_LOGI(TAG, "Response: %s", response);
    return answer;
}
uint8_t send_command(const char *ATcommand, const char *expected1, const char *expected2, uint32_t timeout_ms)
{
    char response[256] = {0};
    int response_len = 0;
    uint8_t byte = 0;
    int len = 0;
    uint8_t result = 0;

    // Clear UART RX buffer
    while (uart_read_bytes(MODEM_UART_PORT, &byte, 1, pdMS_TO_TICKS(10)) > 0);

    // Send the AT command
    uart_write_bytes(MODEM_UART_PORT, ATcommand, strlen(ATcommand));
    uart_write_bytes(MODEM_UART_PORT, "\r\n", 2);
    // ESP_LOGI(TAG, "Sent: %s", ATcommand);


    int64_t start = esp_timer_get_time() / 1000;

    // Read incoming response
    while ((esp_timer_get_time() / 1000 - start) < timeout_ms && response_len < sizeof(response) - 1) {
        len = uart_read_bytes(MODEM_UART_PORT, &byte, 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            response[response_len++] = byte;
            response[response_len] = '\0';

            if (expected1 && strstr(response, expected1)) {
                result = 1;
                break;
            } else if (expected2 && strstr(response, expected2)) {
                result = 2;
                break;
            }
        }
    }
    ESP_LOGI(TAG, "Response: %s", response);
    return result;
}

bool  sim800l_init_gprs(const char *apn) {
    char cmd[128];

    if (!sim800l_send_command("ATE0", "OK", 500)) {
        ESP_LOGE(TAG, "disableEcho command failed");
        return false;
    }
    if (!sim800l_send_command("AT+GSN", "OK", 1000)) {
        // ESP_LOGE(TAG, "IME command failed");
        return false;
    }

    if (!sim800l_send_command("AT+CSMINS?", "+CSMINS: 0,1", 1000)) {
        // ESP_LOGE(TAG, "IME command failed");
        return false;
    }
    currentStatus = NETWORK_STATE_SIM_READY;
    if (!send_command("AT+CREG?", "+CREG: 0,1","+CREG: 0,2", 1000)) {
        // ESP_LOGE(TAG, "AT+CREG? command failed");
        return false;
    }
    currentStatus = NETWORK_STATE_GPRS_ATTACHED;

    return true;

}



bool sim800l_connect_tcp(const char *host, int port) {
    char cmd[128];

    if (!sim800l_send_command("AT+CIPSSL=0", "OK", 5000) ) return false;// no ssl 
    // close all old connections
    if (!sim800l_send_command("AT+CIPSHUT", "SHUT OK", 1000) ) return false;
    // single connection at a time
    if (! sim800l_send_command("AT+CIPMUX=0", "OK", 1000 ) )return false;
    // manually read data
    if (! sim800l_send_command("AT+CIPRXGET=1","OK", 1000) ) return false;

    sprintf(cmd, "AT+CIPSTART=\"TCP\",\"%s\",\"%d\"", host, port);

    if (!sim800l_send_command(cmd, "CONNECT OK", 8000)) {
        ESP_LOGE(TAG, "TCP Connect failed");
        return false;
    }

    ESP_LOGI(TAG, "TCP connection established");
    return true;
}

bool sim800l_send_tcp(const uint8_t *data, int len) {
    char cmd[32];
    sprintf(cmd, "AT+CIPSEND=%d", len);

    if (!sim800l_send_command(cmd, ">", 2000)) {
        ESP_LOGE(TAG, "CIPSEND command failed or prompt not received");
        return false;
    }

    uart_write_bytes(MODEM_UART_PORT, (const char *)data, len);
    uart_write_bytes(MODEM_UART_PORT, "\x1A", 1);  // End with Ctrl+Z

 
    if (!sim800l_send_command("", "SEND OK", 5000)) {
        ESP_LOGE(TAG, "Data send failed");
        return false;
    }

    // ESP_LOGI(TAG, "Data sent successfully");
    return true;
}


// static void mqtt_sim800l_task(void *pvParameters)
// {

//     uint8_t connect_buf[256];

//     while (1) {
//         ESP_LOGI(TAG, "Resetting and reinitializing SIM800L...");

//         if(!sim800l_full_init(MQTT_APN))continue;

//         if (!sim800l_connect_tcp(MQTT_HOST, MQTT_PORT)) {
//             ESP_LOGW(TAG, "TCP connection failed. Will retry...");
//             continue;
//         }

//         // ---- MQTT CONNECT ----
//         int len = mqtt_connect_packet(connect_buf, USER_ID, USER_NAME, USER_PASSWORD);
//         if (!sim800l_send_tcp(connect_buf, len)) {
//             ESP_LOGE(TAG, "MQTT CONNECT failed");
//             sim800l_disconnect_tcp();
//             continue;
//         }

//         uint8_t subscribe_buf[128];
//         int subscribe_len = mqtt_subscribe_packet(subscribe_buf, 1,TOPIC);
//         if (!sim800l_send_tcp(subscribe_buf, subscribe_len)) {
//             ESP_LOGE(TAG, "MQTT SUBSCRIBE failed");
//             sim800l_disconnect_tcp();
//             continue;

//         }
//         // ---- Main loop: PUBLISH/KEEP-ALIVE ----
//         M_payload_t received_payload;
//         uint8_t mqtt_packet[512];  // Buffer for MQTT publish packet

//         while (1) {

//             if (xQueueReceive(modbus_payload_queue, &received_payload, 0) == pdPASS) {
            
//                 int len = mqtt_publish_packet(
//                                 mqtt_packet,
//                                 (const char *)received_payload.topic,
//                                 (const uint8_t *)&received_payload, 
//                                 sizeof(received_payload),
//                                 0 // QOS = 0
//                             );

//                 if (!sim800l_send_tcp(mqtt_packet, len)) {
//                     ESP_LOGE("MQTT", "Failed to publish sensor data");
//                 } else {
//                     ESP_LOGI("MQTT", "Sensor data published successfully");
//                     // save in NVS or flash if needed

//                     break;
//                 }
//             }

//         }

//     }
// }


static void mqtt_sim800l_task(void *pvParameters)
{
    uint8_t connect_buf[256];

    while (1) {
        ESP_LOGI(TAG, "Resetting and reinitializing SIM800L...");

        if (!sim800l_full_init(MQTT_APN)) continue;

        if (!sim800l_connect_tcp(MQTT_HOST, MQTT_PORT)) {
            ESP_LOGW(TAG, "TCP connection failed. Will retry...");
            continue;
        }
        currentStatus = MQTT_STATE_TCP_CONNECTED;

        int len = mqtt_connect_packet(connect_buf, USER_ID, USER_NAME, USER_PASSWORD);
        if (!sim800l_send_tcp(connect_buf, len)) {
            ESP_LOGE(TAG, "MQTT CONNECT failed");
            sim800l_disconnect_tcp();
            continue;
        }
        currentStatus = MQTT_STATE_MQTT_CONNECTED;


        uint8_t subscribe_buf[128];
        int subscribe_len = mqtt_subscribe_packet(subscribe_buf, 1, TOPIC);
        if (!sim800l_send_tcp(subscribe_buf, subscribe_len)) {
            ESP_LOGE(TAG, "MQTT SUBSCRIBE failed");
            sim800l_disconnect_tcp();
            continue;
        }
        currentStatus = MQTT_STATE_SUBSCRIBE;
        ESP_LOGI(TAG, "MQTT Connected and Subscribed");
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for disconnect notice
    }
}

static void mqtt_publish_task(void *pvParameters)
{
    M_payload_t received_payload;
    uint8_t mqtt_packet[512];

    while (1) {
        
        if (xQueueReceive(modbus_payload_queue, &received_payload, portMAX_DELAY) == pdPASS) {
            int len = mqtt_publish_packet(
                mqtt_packet,
                (const char *)received_payload.topic,
                (const uint8_t *)&received_payload,
                sizeof(received_payload),
                0
            );

            if (!sim800l_send_tcp(mqtt_packet, len)) {
                ESP_LOGE("MQTT_PUB", "Failed to publish, notifying reconnect");
                // QUEUE TO THE NVS HERE-----------------------------


                //----------------------------------------------------
                currentStatus = MQTT_STATE_DISCONNECTED;
                xTaskNotifyGive(mqtt_sim800l_task_handle);  // Trigger reconnect
            }

            ESP_LOGI("MQTT_PUB", "Published sensor data");
        }
    }
}



// void mqtt_receive_task(void *pvParameters)
// {
//     char recv_buf[512];

//     while (1) {

//         int len = uart_read_bytes(MODEM_UART_PORT, (uint8_t *)recv_buf, sizeof(recv_buf), pdMS_TO_TICKS(1000));
//         if (len > 0) {
//             mqtt_handle_incoming((uint8_t *)recv_buf, len);
//         }
//         vTaskDelay(pdMS_TO_TICKS(1000));  // Adjust as needed
//     }
// }
void mqtt_sim800l_start(void)
{

    xTaskCreatePinnedToCore(mqtt_sim800l_task, "mqtt_sim800l_task", 1024*4, NULL, 5, &mqtt_sim800l_task_handle,0);
    // xTaskCreatePinnedToCore(mqtt_receive_task, "mqtt_receive_task", 1024*4, NULL, 5, NULL,1);
    xTaskCreatePinnedToCore(mqtt_publish_task, "mqtt_publish_task", 1024*4, NULL, 5, &mqtt_publish_task_handle,0);


}