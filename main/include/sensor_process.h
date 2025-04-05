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

typedef struct {
    size_t head, count;
    sensor_data buffer[];
} queue;

#define prod_enqueue(q_ptr, data) enqueue((queue *) (q_ptr), PRODUCTION_SIZE, (data))
#define fil_enqueue(q_ptr, data) enqueue((queue *) (q_ptr), FILTERED_SIZE, (data))

extern SemaphoreHandle_t mutex_2; // mutex between sensor-processing and sensor-printing tasks
extern SemaphoreHandle_t processing_done;   

extern fil_queue filtered_queue;

void enqueue(queue *queue, size_t size, sensor_data data);                                                                                                                       
void sort_queue(queue *queue_ptr);

void filter_sensor_value(void *pvParameters);
#endif