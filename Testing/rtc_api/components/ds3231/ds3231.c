#include "ds3231.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#define START_REG 0x00

static uint8_t bcd_to_dec(uint8_t val)
{
    return ((val >> 4) * 10) + (val & 0x0F);   
}

static uint8_t dec_to_bcd(uint8_t val) 
{
    return ((val / 10) << 4) | (val % 10);   
}

// I2C write helper
static esp_err_t i2c_write(uint8_t start_reg, uint8_t *data, uint8_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, start_reg, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;  
}

static esp_err_t i2c_read(uint8_t start_reg, uint8_t *data, uint8_t len)
{
    i2c_cmd_handle_t cmd;  

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_WRITE, true); 
    i2c_master_write_byte(cmd, start_reg, true);
    i2c_master_stop(cmd);
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000)));
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, &data[len - 1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret;   
}

// RTC Init
void ds3231_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA,
        .scl_io_num = I2C_MASTER_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(I2C_MASTER_NUM, &conf);
    esp_err_t ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_TX_BUF_DISABLE, I2C_MASTER_RX_BUF_DISABLE, 0);

    if (ret != ESP_OK) {
        printf("[-] Error: installing I2C driver: %s\n", esp_err_to_name(ret)); 
    } else {
        printf("[-] I2C driver installed successfully!\n");
    }
}

// Set time
esp_err_t set_time(Time *time)
{
    uint8_t data[7];

    data[0] = dec_to_bcd(time->seconds);       
    data[1] = dec_to_bcd(time->minutes);
    data[2] = dec_to_bcd(time->hours);
    data[3] = dec_to_bcd(time->day);
    data[4] = dec_to_bcd(time->wday + 1);
    data[5] = dec_to_bcd(time->month + 1); 
    data[6] = dec_to_bcd(time->year - 2000);

    return i2c_write(START_REG, data, sizeof(data));  
}

esp_err_t get_time(Time *current_time)
{
    uint8_t data[7];

    i2c_read(START_REG, data, sizeof(data)); 

    current_time->seconds = bcd_to_dec(data[0]);
    current_time->minutes = bcd_to_dec(data[1]);
    current_time->hours   = bcd_to_dec(data[2]);
    current_time->day     = bcd_to_dec(data[3]);
    current_time->wday    = bcd_to_dec(data[4]);
    current_time->month   = bcd_to_dec(data[5]);
    current_time->year    = bcd_to_dec(data[6]) + 2000;

    // ESP_LOGI(TAG, "Time: %02d-%02d-%04d %02d:%02d:%02d",
    //          current_time->day,
    //          current_time->month,
    //          current_time->year,
    //          current_time->hours,
    //          current_time->minutes,
    //          current_time->seconds);

    return ESP_OK; 
}


void rtc_set_alarm(period_t period, alarm_config_t *config) {
    uint8_t a1m1 = 0, a1m2 = 0, a1m3 = 0, a1m4 = 0;
    uint8_t dy_dt = 0;
    uint8_t day_val = 0;

    switch (period) {
        case HOURLY:
            a1m1 = 0; a1m2 = 0; a1m3 = 1; a1m4 = 1;
            break;
        case DAILY:
            a1m1 = 0; a1m2 = 0; a1m3 = 0; a1m4 = 1;
            break;
        case WEEKLY:
            a1m1 = 0; a1m2 = 0; a1m3 = 0; a1m4 = 0;
            dy_dt = 1;

            day_val = dec_to_bcd(config->dow);
            break;
        case MONTHLY:
        case YEARLY:  // Hardware same as monthly; software check for year
            a1m1 = 0; a1m2 = 0; a1m3 = 0; a1m4 = 0;
            dy_dt = 0;
            day_val = dec_to_bcd(config->date);
            break;
    }

    uint8_t alarm_data[4] = {
        dec_to_bcd(config->sec) | (a1m1 << 7),
        dec_to_bcd(config->min) | (a1m2 << 7),
        dec_to_bcd(config->hour) | (a1m3 << 7),  // 24-hour
        day_val | (a1m4 << 7) | (dy_dt << 6)
    };
    i2c_write(0x07, alarm_data, 4);

    // Set control: A1IE=1, INTCN=1, EOSC=0, A2IE=0
    uint8_t ctrl;
    i2c_read(0x0E, &ctrl, 1);
    ctrl &= ~(1 << 7);  // EOSC=0
    ctrl |= (1 << 0);   // A1IE=1
    ctrl |= (1 << 2);   // INTCN=1
    ctrl &= ~(1 << 1);  // A2IE=0
    i2c_write(0x0E, &ctrl, 1);

    // Clear flags
    clear_alarm_flag();
}

void clear_alarm_flag(void) {
    uint8_t status;
    i2c_read(0x0F, &status, 1);
    status &= ~0x01;  // Clear A1F (bit 0)
    i2c_write(0x0F, &status, 1);
}
