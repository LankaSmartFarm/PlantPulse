
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mqtt.h"

void HALT(){

    while (1)
    {
        /* code */
    }
    
}


void app_main(void) {
    
    // Initialize GPIO for SIM800L
    sim800l_gpio_init();
    // Initialize UART for SIM800L communication
    sim800l_uart_init();
    // Start the MQTT task
    mqtt_sim800l_start();
}