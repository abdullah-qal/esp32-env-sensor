#ifndef SENSOR_PROCESS_H
#define SENSOR_PROCESS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensor_config.h"

#define FILTERED_SIZE 10

typedef struct {
    size_t head, count;
    sensor_data buffer[FILTERED_SIZE];
} fil_queue;

extern SemaphoreHandle_t mutex_2; // mutex between sensor-processing and sensor-printing tasks
extern SemaphoreHandle_t processing_done;   

extern fil_queue filtered_queue;

void insert_into_production_queue(sensor_data data);
void insert_into_filtered_queue(sensor_data data);
void sort_queue(void *queue_ptr);

void filter_sensor_value(void *pvParameters);
#endif