
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "esp_log.h"
#include "mqtt_client.h"
#include "sim800l.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#define MQTT_APN     "internet"
#define MQTT_HOST    "mqtt.lankasmart.farm"
#define MQTT_PORT    1883
#define RECONNECT_DELAY_MS 5000

static const char *TAG = "MQTT_TASK";
const char *myTopic = "test/topic";
const char *msg = "Hello MQTT";

extern SemaphoreHandle_t uart_mutex;


static uint16_t mqtt_packet_id = 1;  // Global packet ID counter


#define MQTT_KEEPALIVE 60

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



static void mqtt_sim800l_task(void *pvParameters)
{

    uint8_t connect_buf[256];

    while (1) {
        ESP_LOGI(TAG, "Resetting and reinitializing SIM800L...");

        if(!sim800l_full_init(MQTT_APN))continue;

        if (!sim800l_connect_tcp(MQTT_HOST, MQTT_PORT)) {
            ESP_LOGW(TAG, "TCP connection failed. Will retry...");
            continue;
        }

        // ---- MQTT CONNECT ----
        int len = mqtt_connect_packet(connect_buf, "pjj2wavynvq62q9poz87", "a3unrrv00hocglccoxy2", "m6a7got5mywyhony2hmv");
        if (!sim800l_send_tcp(connect_buf, len)) {
            ESP_LOGE(TAG, "MQTT CONNECT failed");
            sim800l_disconnect_tcp();
            continue;
        }

        uint8_t subscribe_buf[128];
        int subscribe_len = mqtt_subscribe_packet(subscribe_buf, 1,"v1/devices/me/telemetry");
        if (!sim800l_send_tcp(subscribe_buf, subscribe_len)) {
            ESP_LOGE(TAG, "MQTT SUBSCRIBE failed");
            sim800l_disconnect_tcp();
            continue;

        }
        // ---- Main loop: PUBLISH/KEEP-ALIVE ----
        while (1) {

            static uint8_t failCounter = 0;

            for (int i = 0; i < 5; i++) {

                if(!publish_sensor_data(
                    25.5 + i,      // Temperature
                    40.0 + i,      // Moisture
                    6.5 + i * 0.1, // pH
                    500 + i * 10,  // Conductivity
                    20 + i,        // N
                    15 + i,        // P
                    30 + i,        // K
                    350 + i * 5,   // Salinity
                    450 + i * 10   // TDS
                )){
                    failCounter++;
                    if(failCounter>9){
                        break; // Exit the loop to reconnect
                    } else {
                        ESP_LOGW(TAG, "Failed to publish sensor data, retrying...%d", failCounter);
                    }
                    
                }else failCounter = 0; // Reset fail counter on success

            }
            vTaskDelay(pdMS_TO_TICKS(1000));  // simulate keep-alive
            if(failCounter>9){
                ESP_LOGE(TAG, "Failed to publish sensor data after multiple attempts. Disconnecting...");
                break; // Exit the loop to reconnect
            }

        }

    }
}



bool  publish_sensor_data(float temp, float moisture, float pH, uint16_t cond,
                         uint16_t N, uint16_t P, uint16_t K,
                         uint16_t salinity, uint16_t tds)
{
    const char *topic = "v1/devices/me/telemetry";
    char payload[256];
    uint8_t packet[512];

    snprintf(payload, sizeof(payload),
             "{\"temperature\":%.1f,\"moisture\":%.1f,\"pH\":%.1f,"
             "\"conductivity\":%u,\"nitrogen\":%u,\"phosphorus\":%u,"
             "\"potassium\":%u,\"salinity\":%u,\"tds\":%u}",
             temp, moisture, pH, cond, N, P, K, salinity, tds);

    ESP_LOGI("MQTT", "Publishing: %s", payload);

    int len = mqtt_publish_packet(packet, topic, (const uint8_t *)payload, strlen(payload), 0);

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


void mqtt_receive_task(void *pvParameters)
{
    char recv_buf[512];

    while (1) {
        if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(50))) {

            int len = uart_read_bytes(MODEM_UART_PORT, (uint8_t *)recv_buf, sizeof(recv_buf), pdMS_TO_TICKS(1000));
            if (len > 0) {
                mqtt_handle_incoming((uint8_t *)recv_buf, len);
            }
            xSemaphoreGive(uart_mutex);
        }else {
            ESP_LOGW(TAG, "UART mutex not available for receive task");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  // Adjust as needed
    }
}

void mqtt_sim800l_start(void)
{

    xTaskCreatePinnedToCore(mqtt_sim800l_task, "mqtt_sim800l_task", 1024*4, NULL, 5, NULL,0);
    // xTaskCreatePinnedToCore(mqtt_receive_task, "mqtt_receive_task", 1024*4, NULL, 5, NULL,1);

}








// {clientId:"yv4ligfrp7izgjgzw3bw",userName:"ceu55mcsg21m3k7ayun2",password:"pr7038gzya3ep21blbjy"}    60c2b530-3534-11f0-8e3c-efdd6beb784f