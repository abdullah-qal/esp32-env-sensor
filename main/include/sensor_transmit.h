#ifndef SENSOR_TRANSMIT_H
#define SENSOR_TRANSMIT_H

#include "sensor_process.h"
#include "sensor_config.h"

#define TRANSMISSION_PERIOD 1000 // in ms

typedef struct {
    sensor_data min, max, median, stddev;
} ble_data;

ble_data calculate_data(fil_queue queue);
void print_sensor_value(void *pvParameters);

#endif