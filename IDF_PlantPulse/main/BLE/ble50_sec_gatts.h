/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef BLE50_SEC_GATTS_H
#define BLE50_SEC_GATTS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dataLogging.h"
/* Attributes State Machine */
enum
{
    IDX_SVC,
    IDX_CHAR_A,
    IDX_CHAR_VAL_A,
    IDX_CHAR_CFG_A,

    IDX_CHAR_B,
    IDX_CHAR_VAL_B,

    IDX_CHAR_C,
    IDX_CHAR_VAL_C,

    HRS_IDX_NB,
};


static const char *TAG = "DEVICE_ID";

void BLEStart(void);
void sendOverBLE_soil_packet(soil_packet_t soil_packet,uint16_t total_len);
void sendOverBLE_ping_packet(ping_packet_t ping_packet, uint16_t total_len);
void get_device_id(uint8_t *device_id);


extern TaskHandle_t soilBleTaskHandle;
extern TaskHandle_t pingBleTaskHandle;

#endif