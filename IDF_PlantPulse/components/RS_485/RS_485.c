#include <stdio.h>
#include "RS_485.h"


static const char *TAG = "MB_RTU";

// Initialize Modbus RTU UART
void RS_485_Init(void) {
    uart_config_t uart_config = {
        .baud_rate = MB_BAUD_RATE,
        .data_bits = MB_DATA_BITS,
        .parity = MB_PARITY,
        .stop_bits = MB_STOP_BITS,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(MB_UART_PORT, &uart_config));
    // Set UART pins
    ESP_ERROR_CHECK(uart_set_pin(MB_UART_PORT, MB_TX_PIN, MB_RX_PIN, MB_RTS_PIN, MB_CTS_PIN));
    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(MB_UART_PORT, MB_RX_BUFFER_SIZE, MB_TX_BUFFER_SIZE, 0, NULL, 0));
    ESP_LOGI(TAG, "MB RTU UART initialized");

}

// Send Modbus RTU frame
void MB_rtu_send(uint8_t slave_address, uint8_t function_code, uint16_t start_address, uint16_t quantity, uint8_t *data, uint16_t data_length) {
    uint8_t frame[256];
    uint16_t frame_length = 0;

    // Build Modbus RTU frame
    frame[frame_length++] = slave_address;
    frame[frame_length++] = function_code;
    frame[frame_length++] = (start_address >> 8) & 0xFF;
    frame[frame_length++] = start_address & 0xFF;
    frame[frame_length++] = (quantity >> 8) & 0xFF;
    frame[frame_length++] = quantity & 0xFF;

    // Add data if present
    if (data != NULL && data_length > 0) {
        for (uint16_t i = 0; i < data_length; i++) {
            frame[frame_length++] = data[i];
        }
    }

    // Calculate CRC
    uint16_t crc = crc16(frame, frame_length);
    frame[frame_length++] = crc & 0xFF;
    frame[frame_length++] = (crc >> 8) & 0xFF;

    uart_write_bytes(MB_UART_PORT, (const char *)frame, frame_length);
    // ESP_LOGI(TAG, "Modbus RTU frame sent");
   
}

uint8_t MB_rtu_receive(uint8_t *response_buffer, uint16_t buffer_size) {
    int length = uart_read_bytes(MB_UART_PORT, response_buffer, buffer_size, pdMS_TO_TICKS(100));
    if (length > 0) {
        // ESP_LOGI(TAG, "Received raw data:");
        // for (int i = 0; i < length; i++) {
        //     printf("%02X ", response_buffer[i]);
        // }

        // Verify CRC
        uint16_t crc_received = (response_buffer[length - 1] << 8) | response_buffer[length - 2];
        uint16_t crc_calculated = crc16(response_buffer, length - 2);

        if (crc_received == crc_calculated) {
            // ESP_LOGI(TAG, "Modbus RTU frame received with valid CRC");
            return 1; // Valid frame
        } else {
            // ESP_LOGE(TAG, "Modbus RTU frame received with invalid CRC");
            return 0; // Invalid frame
        }
    }
    // ESP_LOGE(TAG, "No data received");
    return 0; // No data received
}

// Calculate CRC16 for Modbus RTU
uint16_t crc16(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}


