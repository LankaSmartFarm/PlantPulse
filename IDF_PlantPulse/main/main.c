
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dataLogging.h"
#include "mqtt.h"

extern TaskHandle_t modbusTaskHandle;
extern QueueHandle_t modbus_payload_queue;

void HALT(){

    while (1)
    {
    }
    
}


void app_main(void) {

    modbus_payload_queue = xQueueCreate(5, sizeof(M_payload_t));

    
    // Initialize GPIO for SIM800L
    sim800l_gpio_init();
    // Initialize UART for SIM800L communication
    sim800l_uart_init();
    // Start the MQTT task
    mqtt_sim800l_start();

    xTaskCreatePinnedToCore(modbus_collect_task, "data_collect_task", 1024*4, NULL, 5, &modbusTaskHandle,0);



    
}