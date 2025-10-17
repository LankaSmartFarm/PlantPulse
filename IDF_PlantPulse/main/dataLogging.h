#ifndef DATA_LOGGING_H
#define DATA_LOGGING_H
#include <stdint.h>
#include <stdbool.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "RS_485.h"
#include "mqtt.h"


#define SENSOR_COUNT 20
#define SENSOR_SLAVE_START 1
#define SOIL8IN1_FUNC_CODE 0x03
#define SOIL8IN1_REG_ADDR 0x0000
#define SOIL8IN1_REG_COUNT     8    // temperature, humidity, EC, pH, N, P, K, moisture


// Define your Modbus constants
#define SLAVE_ADDR_FILE "modbus_slave_addr.bin"
#define SOIL_PACKET_SIZE  64

// Unique 32-bit packet identifiers (undisplayable / non-ASCII sequences)
#define PING_PACKET_TYPE   0x01F2A3C4   // Ping status packet (ESP32 → App)
#define SOIL_PACKET_TYPE   0x02B4C6D8   // Soil sensor packet (ESP32 → App)
#define ACK_PACKET_TYPE    0x03D7E9FA   // Acknowledgment / response (App → ESP32)
#define CMD_PACKET_TYPE    0x04EAFB1C   // Command / control (App → ESP32)
#define SLAVE_ADDR_PACKET_TYPE 0x05ACBDDE // Set Modbus slave address (App → ESP32)



#define CHARGE_DETECT_GPIO  GPIO_NUM_2
#define BATTERY_CHARGE_STATE GPIO_NUM_1
#define BATTERY_ADC_CHANNEL ADC1_CHANNEL_0
#define DEFAULT_VREF 1100         
#define NO_OF_SAMPLES 64        
#define MIN_VOLTAGE 1600




typedef struct __attribute__((packed)) {
    uint16_t temperature;   // °C × 100
    uint16_t humidity;      // % × 100
    uint16_t ec;            // µS/cm
    uint16_t ph;            // ×100
    uint16_t nitrogen;      // mg/kg or ppm
    uint16_t phosphorus;    // mg/kg or ppm
    uint16_t potassium;     // mg/kg or ppm
    uint16_t soil_moisture; // % × 100
} soil8in1_data_t;

// 🌾 Main soil packet (total 64 bytes)
typedef struct __attribute__((packed)) {
    uint32_t data_type;        // 4 bytes
    uint8_t  device_id[6];     // 6 bytes
    soil8in1_data_t soil8in1;  // 16 bytes
    uint8_t  reserved[30];     // 30 bytes (0xFF fill to reach 64 bytes)
    uint16_t crc;              // 2 bytes

} soil_packet_t;

// 📡 Ping packet structure (total 32 bytes)
typedef struct __attribute__((packed)) {
    uint32_t packet_type;     // 4 bytes (PING_PACKET_TYPE)
    uint8_t  device_id[6];    // 6 bytes (ESP32 MAC or custom ID)
    uint32_t   runTime;        // 1 byte  (RSSI value)
    uint16_t battery_level;   // 2 bytes (mV or scaled)
    uint8_t  charge_status;   // 1 byte  (0 = not charging, 1 = charging)
    uint8_t  modbus_slave_id; // 1 byte
    uint8_t  reserved[16];    // 16 bytes reserved (filled with 0xFF)
    uint16_t crc;             // 2 bytes (CRC16)
} ping_packet_t;




void dataLoggingHandleTask(void *pvParameters);
void build_soil_packet(soil_packet_t *packet);
void soil_ble_task(void *pv);
void ping_ble_task(void *pv);
extern void sendOverBLE_soil_packet(soil_packet_t soil_packet,uint16_t total_len);
extern void sendOverBLE_ping_packet(ping_packet_t ping_packet, uint16_t total_len);



#endif // DATA_LOGGING_H