#include "myRtc.h"

static const char *TAG = "DS3231";

TaskHandle_t rtcTask_Handle = NULL;

void rtcInit(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE,
                       I2C_MASTER_TX_BUF_DISABLE, 0);
}

uint8_t bcd2dec(uint8_t val)
{
    return (val >> 4) * 10 + (val & 0x0F);
}

uint8_t dec2bcd(uint8_t val)
{
    return ((val / 10) << 4) | (val % 10);
}
esp_err_t getTime(struct tm *timeinfo)
{
    uint8_t data[7];
    i2c_master_write_read_device(I2C_MASTER_NUM, DS3231_ADDR,
                                 (uint8_t[]){0x00}, 1, data, 7, 1000 / portTICK_PERIOD_MS);

    timeinfo->tm_sec = bcd2dec(data[0] & 0x7F);
    timeinfo->tm_min = bcd2dec(data[1]);
    timeinfo->tm_hour = bcd2dec(data[2] & 0x3F); // 24-hour format
    timeinfo->tm_wday = bcd2dec(data[3]);
    timeinfo->tm_mday = bcd2dec(data[4]);
    timeinfo->tm_mon = bcd2dec(data[5] & 0x1F) - 1; // tm_mon = 0–11
    timeinfo->tm_year = bcd2dec(data[6]) + 100;     // tm_year = years since 1900

    return ESP_OK;
}
esp_err_t setTime(struct tm *timeinfo)
{
    uint8_t data[8];
    data[0] = 0x00; // starting register
    data[1] = dec2bcd(timeinfo->tm_sec);
    data[2] = dec2bcd(timeinfo->tm_min);
    data[3] = dec2bcd(timeinfo->tm_hour);
    data[4] = dec2bcd(timeinfo->tm_wday);
    data[5] = dec2bcd(timeinfo->tm_mday);
    data[6] = dec2bcd(timeinfo->tm_mon + 1);
    data[7] = dec2bcd(timeinfo->tm_year - 100);

    return i2c_master_write_to_device(I2C_MASTER_NUM, DS3231_ADDR, data, 8,
                                      1000 / portTICK_PERIOD_MS);
}

static void ds3231_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t data[2] = {reg, val};
    i2c_master_write_to_device(I2C_MASTER_NUM, DS3231_ADDR, data, 2, 1000 / portTICK_PERIOD_MS);
}

// Read single register
static uint8_t ds3231_read_reg(uint8_t reg)
{
    uint8_t val;
    i2c_master_write_read_device(I2C_MASTER_NUM, DS3231_ADDR, &reg, 1, &val, 1, 1000 / portTICK_PERIOD_MS);
    return val;
}
// Enable Alarm2 every hour (on minute = 00)
void setAlarmHour(void)
{
    uint8_t alarm_data[4];
    alarm_data[0] = 0x0B; // starting register (Alarm2 minutes)
    alarm_data[1] = 0x00; // Minutes = 00, MSB=0 (must match)
    alarm_data[2] = 0x80; // Hours = don't care (MSB=1)
    alarm_data[3] = 0x80; // Day/Date = don't care (MSB=1)

    i2c_master_write_to_device(I2C_MASTER_NUM, DS3231_ADDR, alarm_data, 4, 1000 / portTICK_PERIOD_MS);

    // Enable Alarm2 interrupt
    uint8_t ctrl = ds3231_read_reg(DS3231_CTRL_REG);
    ctrl |= (1 << 1); // A2IE
    ctrl |= (1 << 2); // INTCN
    ds3231_write_reg(DS3231_CTRL_REG, ctrl);

    // Clear Alarm2 flag
    uint8_t status = ds3231_read_reg(DS3231_STATUS_REG);
    status &= ~(1 << 1);
    ds3231_write_reg(DS3231_STATUS_REG, status);

    ESP_LOGI(TAG, "Alarm2 set: trigger every hour on minute 00");
    ESP_LOGI(TAG, "Waiting for DS3231 alarm interrupts...");
}
static void ds3231_clear_alarm2_flag(void)
{
    uint8_t status = ds3231_read_reg(DS3231_STATUS_REG);
    status &= ~(1 << 1); // Clear A2F
    ds3231_write_reg(DS3231_STATUS_REG, status);
}
// Configure Alarm2 to trigger every 1 minute
void setAlarmMinute(void)
{
    uint8_t alarm_data[4];
    alarm_data[0] = 0x0B; // Alarm2 start register
    alarm_data[1] = 0x80; // Minute "don't care" → triggers every minute
    alarm_data[2] = 0x80; // Hour "don't care"
    alarm_data[3] = 0x80; // Day/Date "don't care"

    i2c_master_write_to_device(I2C_MASTER_NUM, DS3231_ADDR, alarm_data, 4, 1000 / portTICK_PERIOD_MS);

    // Enable Alarm2 interrupt
    uint8_t ctrl = ds3231_read_reg(DS3231_CTRL_REG);
    ctrl |= (1 << 1); // A2IE enable
    ctrl |= (1 << 2); // INTCN enable (use INT pin)
    ds3231_write_reg(DS3231_CTRL_REG, ctrl);

    // Clear Alarm2 flag
    ds3231_clear_alarm2_flag();

    ESP_LOGI(TAG, "Alarm2 set to trigger every minute");
    ESP_LOGI(TAG, "Waiting for DS3231 alarm interrupts...");
}

void isrInit(void)
{

  // Setup GPIO interrupt for INT pin
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE, // DS3231 INT goes LOW
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << INT_PIN,
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);

    // Install ISR service and attach handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(INT_PIN, rtc_isr_handler, NULL);

}
// ISR: just notify
static void IRAM_ATTR rtc_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Notify the RTC task
    vTaskNotifyGiveFromISR(rtcTask_Handle, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

// Task: handle alarm safely
void rtcTask(void *pvParameters)
{

    setAlarmMinute();
    // setAlarmHour();
    while (1)
    {
        // Wait for ISR notification
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // Now safe to use I2C
        ds3231_clear_alarm2_flag();
        ESP_LOGI("MAIN", "NOTIFY TO LOGGING DATA");
        xTaskNotifyGive(dataLoggingTask_Handle);
    }
}
