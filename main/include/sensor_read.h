#ifndef SENSOR_READ_H
#define SENSOR_READ_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensor_config.h"

#define PRODUCTION_SIZE 5 // Don't set too large

typedef struct {
    size_t head, count;
    sensor_data buffer[PRODUCTION_SIZE];
} prod_queue;

extern SemaphoreHandle_t mutex_1; // mutex between sensor-reading and sensor-processing tasks
extern SemaphoreHandle_t reading_done;    

extern prod_queue production_queue;

void i2c_sensor_init(void);
void i2c_sensor_read(void *pvParameters);

#endif