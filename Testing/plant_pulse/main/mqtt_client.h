
#pragma once

#include <stdint.h>



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

