
#ifndef MQTT_H
#define MQTT_H

#include "string.h"
#include <stdint.h>
#include <stdbool.h>
#include "stdio.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define MODEM_UART_PORT UART_NUM_1
#define MODEM_UART_TX (GPIO_NUM_16)
#define MODEM_UART_RX (GPIO_NUM_15)
#define MODEM_RESET_PIN (GPIO_NUM_3)

typedef enum
{
    // Sim Network States
    NETWORK_STATE_OFF = 0,       // SIM800L powered off or not initialized
    NETWORK_STATE_SIM_READY,     // SIM card detected and ready
    NETWORK_STATE_GPRS_ATTACHED, // Successfully attached to GPRS
    NETWORK_STATE_GPRS_ERROR,    // Failed to attach to GPRS
                              // MQTT-Specific States
    MQTT_STATE_DISCONNECTED,   // TCP/MQTT not connected
    MQTT_STATE_TCP_CONNECTED,  // TCP connected (but MQTT not yet)
    MQTT_STATE_MQTT_CONNECTED, // Fully connected to MQTT broker
    MQTT_STATE_SUBSCRIBE,      // Subscribing to a topic
    MQTT_STATE_ERROR,          // General MQTT/Network error
} networkState;

typedef struct
{
    uint64_t timestamp;   // 8 bytes
    uint8_t device_id[6]; // 6 bytes (e.g., MAC address)
    char topic[20];       // 20 bytes
    uint16_t battery_mv;  // 2 bytes

    struct
    {
        uint16_t ph;           // 2
        uint16_t moisture;     // 2
        int16_t temperature;   // 2
        uint16_t conductivity; // 2
        uint16_t nitrogen;     // 2
        uint16_t phosphorus;   // 2
        uint16_t potassium;    // 2
    } sensor_data[20];         // 14 × 20 = 280 bytes

    uint8_t reserved[4]; // 4 bytes (can extend if needed)
    uint16_t crc;        // 2 bytes
} __attribute__((packed)) M_payload_t;

extern QueueHandle_t modbus_payload_queue;


void initTask(void);
void initGPIO(void);
void simHardHeset(uint8_t wait_seconds);
void initUART(void);
bool initGPRS(const char *apn);
void sendCmd_0(const char *cmd);
uint8_t sendCmd_01(const char *ATcommand, const char *expected_answer, uint32_t timeout_ms);
uint8_t send_command_02(const char *ATcommand, const char *expected1, const char *expected2, uint32_t timeout_ms);

bool connectTCP(const char *host, int port);
bool sendTCP(const uint8_t *data, int len);
bool mqttConnect(const char *client_id, const char *username, const char *password);
int mqttPublishPack(uint8_t *packet, const char *topic, const uint8_t *data, uint16_t data_len, uint8_t qos);
bool mqttSubscribe(uint16_t packet_id, const char *topic);
void mqtt_callback(char *topic, uint8_t *payload, unsigned int len);
void mqtt_process_packet(uint8_t *data, int len, void (*mqtt_callback)(char *, uint8_t *, unsigned int));
bool mqtt_sim800l_loop(void (*mqtt_callback)(char *topic, uint8_t *payload, unsigned int len));
int readResponse(char *resp_buf, size_t max_len, int timeout_ms);

extern void dataLoggingHandleTask(void *pvParameters);

#endif