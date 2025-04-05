#include "esp_log.h"

#include "include/sensor_read.h"
#include "include/sensor_process.h"

static const char *SENSOR_TAG = "SENSOR_PROCESS";

SemaphoreHandle_t mutex_2;
SemaphoreHandle_t processing_done;

fil_queue filtered_queue = {0};

void enqueue(queue *queue, size_t size, sensor_data data) {
    size_t raw_size = sizeof(queue->buffer[queue->head].raw) / sizeof(queue->buffer[queue->head].raw[0]);

    for (size_t i = 0; i < raw_size; ++i)
        queue->buffer[queue->head].raw[i] = data.raw[i];

    queue->head = (queue->head + 1) % size;
    if (queue->count < size)
        queue->count++;
}

void sort_queue(queue *queue) {
    size_t raw_size = sizeof(queue->buffer[queue->head].raw) / sizeof(queue->buffer[queue->head].raw[0]);
    for (size_t i = 0; i < raw_size; ++i) { // Loop through all data streams (e.g., ax, ay, ...)
        for (size_t j = 1; j < queue->count; ++j) { // Loop through all data values of that data stream
            int32_t key = (int32_t)queue->buffer[j].raw[i];
            size_t k = j - 1;
            for (; k != SIZE_MAX && (int32_t)queue->buffer[k].raw[i] > key; --k) 
                queue->buffer[k + 1].raw[i] = queue->buffer[k].raw[i];  
            queue->buffer[k + 1].raw[i] = key;
        }
    }
}

void filter_sensor_value(void *pvParameters) {
    while (1) {
        xSemaphoreTake(reading_done, portMAX_DELAY);

        xSemaphoreTake(mutex_1, portMAX_DELAY);
        prod_queue sorted_queue = production_queue;
        xSemaphoreGive(mutex_1);
        
        sort_queue((queue *)&sorted_queue);

        xSemaphoreTake(mutex_2, portMAX_DELAY);
        fil_enqueue(&filtered_queue, sorted_queue.buffer[sorted_queue.count / 2]);
        xSemaphoreGive(mutex_2);
        
        xSemaphoreGive(processing_done);
    }
}