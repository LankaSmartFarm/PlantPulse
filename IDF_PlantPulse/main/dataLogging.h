#ifndef DATA_LOGGING_H
#define DATA_LOGGING_H
#include <stdint.h>
#include <stdbool.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "RS_485.h"
#include "mqtt.h"


#define SENSOR_COUNT 20
#define SENSOR_SLAVE_START 1
#define SENSOR_FUNC_CODE 0x03
#define SENSOR_REG_ADDR 0x0000
#define SENSOR_REG_COUNT 7  // 7 registers, 2 bytes each

// extern uint64_t get_unix_timestamp(void);
// extern void get_device_id(uint8_t *buf);
// extern uint16_t get_battery_mv(void);


void dataLoggingHandleTask(void *pvParameters);


#endif // DATA_LOGGING_H