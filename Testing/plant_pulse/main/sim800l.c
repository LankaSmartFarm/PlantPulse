
#include "driver/uart.h"
#include "esp_log.h"
#include "sim800l.h"
#include <string.h>
#include "driver/gpio.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "projConfig.h"
#include <stdint.h>

static uint16_t mqtt_packet_id = 1;  // Global packet ID counter
SemaphoreHandle_t uart_mutex;

#define TAG "SIM800L"




void sim800l_disconnect_tcp(void) {
    sim800l_send_command("AT+CIPCLOSE", "CLOSE OK", 3000);
}
bool  sim800l_full_init(const char *apn) {
       

    // if (sim800l_send_command("AT+CREG?", "+CREG: 0,1", 1000)) {//2
    //     ESP_LOGE(TAG, "AT+CREG? command failed");
    //     sim800l_hard_reset(10);
    // }
    uint8_t max_try= 3;
    while ( send_command("AT+CREG?", "+CREG: 0,1" ,"+CREG: 0,2", 1000) == 0 &&  max_try>0) {
        max_try--;
    }
    if (max_try==0) {
        sim800l_hard_reset(10);
        return sim800l_init_gprs(apn);
    }else return true;
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
    ESP_LOGI(TAG, "Resetting SIM800L via MODEM_RESET_PIN...");
    gpio_set_level(MODEM_RESET_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(500));   // hold LOW for 500ms
    gpio_set_level(MODEM_RESET_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(wait_seconds*1000)); // wait 10 seconds after reset
    ESP_LOGI(TAG, "SIM800L reset done");
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

    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(1000))) {
        uart_write_bytes(MODEM_UART_PORT, cmd, strlen(cmd));
        uart_write_bytes(MODEM_UART_PORT, "\r\n", 2);
        xSemaphoreGive(uart_mutex);

        ESP_LOGI(TAG, "Sent: %s", cmd);
    } else {
        ESP_LOGW(TAG, "UART mutex not available for send_cmd");
    }
}


int sim800l_wait_response(char *resp_buf, size_t max_len, int timeout_ms) {

    int len = 0;
    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(1000))) {

        len = uart_read_bytes(MODEM_UART_PORT, (uint8_t*)resp_buf, max_len - 1, pdMS_TO_TICKS(timeout_ms));
        if (len > 0) {
            resp_buf[len] = 0;
            ESP_LOGI(TAG, "Resp: %s", resp_buf);
        }
        xSemaphoreGive(uart_mutex);
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
    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(5))) {

        while (uart_read_bytes(MODEM_UART_PORT, &byte, 1, pdMS_TO_TICKS(10)) > 0);
        xSemaphoreGive(uart_mutex);
    }else {
        ESP_LOGW(TAG, "UART mutex not available");
    }

    // Send the AT command

    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(10))) {

        uart_write_bytes(MODEM_UART_PORT, ATcommand, strlen(ATcommand));
        uart_write_bytes(MODEM_UART_PORT, "\r\n", 2);
        ESP_LOGI(TAG, "Sent: %s", ATcommand);

        xSemaphoreGive(uart_mutex);
    }else {
        ESP_LOGW(TAG, "UART mutex not available");
    }

    int64_t start = esp_timer_get_time() / 1000;


    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(10))) {

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
        ESP_LOGI(TAG, "Response: %s", response);
        xSemaphoreGive(uart_mutex);
    }else {
        ESP_LOGW(TAG, "UART mutex not available ");
    }
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
    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(10))) {
        while (uart_read_bytes(MODEM_UART_PORT, &byte, 1, pdMS_TO_TICKS(10)) > 0);
        xSemaphoreGive(uart_mutex);
    } else {
        ESP_LOGW(TAG, "UART mutex not available (clear)");
    }

    // Send the AT command
    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(10))) {
        uart_write_bytes(MODEM_UART_PORT, ATcommand, strlen(ATcommand));
        uart_write_bytes(MODEM_UART_PORT, "\r\n", 2);
        ESP_LOGI(TAG, "Sent: %s", ATcommand);
        xSemaphoreGive(uart_mutex);
    } else {
        ESP_LOGW(TAG, "UART mutex not available (write)");
    }

    int64_t start = esp_timer_get_time() / 1000;

    // Read incoming response
    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(10))) {
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
        xSemaphoreGive(uart_mutex);
    } else {
        ESP_LOGW(TAG, "UART mutex not available (read)");
    }

    return result;
}

bool  sim800l_init_gprs(const char *apn) {
    char cmd[128];

    if (!sim800l_send_command("ATE0", "OK", 500)) {
        ESP_LOGE(TAG, "disableEcho command failed");
        return false;
    }
    if (!sim800l_send_command("AT+GSN", "OK", 1000)) {
        ESP_LOGE(TAG, "IME command failed");
        return false;
    }

    if (!sim800l_send_command("AT+CSMINS?", "+CSMINS: 0,1", 1000)) {
        ESP_LOGE(TAG, "IME command failed");
        return false;
    }

    if (!send_command("AT+CREG?", "+CREG: 0,1","+CREG: 0,2", 1000)) {
        ESP_LOGE(TAG, "AT+CREG? command failed");
        return false;
    }
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
        if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(1000))) {

            uart_write_bytes(MODEM_UART_PORT, (const char *)data, len);
            uart_write_bytes(MODEM_UART_PORT, "\x1A", 1);  // End with Ctrl+Z

            xSemaphoreGive(uart_mutex);
        }
    if (!sim800l_send_command("", "SEND OK", 5000)) {
        ESP_LOGE(TAG, "Data send failed");
        return false;
    }
    return true;
}
