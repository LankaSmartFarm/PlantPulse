
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/uart.h"
#include "esp_log.h"

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
