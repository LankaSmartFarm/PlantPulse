#ifndef RTC_H
#define RTC_H

#include <stdio.h>
#include "driver/i2c.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include <esp_err.h>
#include <time.h>

#define I2C_MASTER_SCL_IO 5
#define I2C_MASTER_SDA_IO 4
#define I2C_MASTER_NUM I2C_NUM_0
#define DS3231_ADDR 0x68
#define DS3231_CTRL_REG 0x0E
#define DS3231_STATUS_REG 0x0F
#define I2C_MASTER_FREQ_HZ 100000
#define DS3231_ALARM1_REG 0x07
#define INT_PIN 6 // GPIO connected to INT/SQW

#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0



extern TaskHandle_t dataLoggingTask_Handle;


void isrInit(void);

static void IRAM_ATTR rtc_isr_handler(void *arg);
void rtcTask(void *pvParameters);
void rtcInit(void);
esp_err_t setTime(struct tm *timeinfo);
esp_err_t getTime(struct tm *timeinfo);
void setAlarmHour(void);
void setAlarmMinute(void);





#endif