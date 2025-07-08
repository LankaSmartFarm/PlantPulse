
// #ifndef MQTT_H
// #define MQTT_H


#include <stdbool.h>
#include <stdint.h>
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define MODEM_UART_PORT      UART_NUM_1
#define MODEM_UART_TX        (GPIO_NUM_16)
#define MODEM_UART_RX         (GPIO_NUM_15)
#define MODEM_RESET_PIN       (GPIO_NUM_3)

void sim800l_gpio_init(void);
void  sim800l_hard_reset(uint8_t wait_seconds);
void sim800l_uart_init(void);
bool sim800l_init_gprs(const char *apn);
bool sim800l_connect_tcp(const char *host, int port);
bool sim800l_send_tcp(const uint8_t *data, int len);
void sim800l_disconnect_tcp(void);
uint8_t sim800l_send_command(const char *ATcommand, const char *expected_answer, uint32_t timeout_ms);
uint8_t send_command(const char *ATcommand, const char *expected1, const char *expected2, uint32_t timeout_ms);



void mqtt_sim800l_start(void);
int mqtt_connect_packet(uint8_t *buf, const char *client_id, const char *username, const char *password);
int mqtt_subscribe_packet(uint8_t *buf, uint16_t packet_id, const char *topic);
void mqtt_receive_task(void *pvParameters);
bool publish_sensor_data(float temp, float moisture, float pH, uint16_t cond,
                         uint16_t N, uint16_t P, uint16_t K,
                         uint16_t salinity, uint16_t tds);


int mqtt_publish_packet(uint8_t *packet, const char *topic, const uint8_t *data, uint16_t data_len, uint8_t qos);
static uint8_t *mqtt_encode_string(uint8_t *buf, const char *str);
void mqtt_handle_incoming(const uint8_t *data, int len);
    

extern bool  sim800l_full_init(const char *apn) ;




// #endif