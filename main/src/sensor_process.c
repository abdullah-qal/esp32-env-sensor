#include "esp_log.h"

#include "include/sensor_read.h"
#include "include/sensor_process.h"

static const char *SENSOR_TAG = "SENSOR_PROCESS";

SemaphoreHandle_t mutex_2;
SemaphoreHandle_t processing_done;

fil_queue filtered_queue = {0};

void insert_into_production_queue(sensor_data data) {
    // ESP_LOGI("QUEUE", "insert_into_production_queue called");

    // if (queue == NULL) {
    //     ESP_LOGE("QUEUE", "Queue pointer is NULL!");
    //     return;
    // }

    // ESP_LOGI("QUEUE", "Queue head = %d, count = %d", queue->head, queue->count);

    size_t raw_size = sizeof(production_queue.buffer[production_queue.head].raw) / sizeof(production_queue.buffer[production_queue.head].raw[0]);

    for (size_t i = 0; i < raw_size; ++i)
        production_queue.buffer[production_queue.head].raw[i] = data.raw[i];

    production_queue.head = (production_queue.head + 1) % PRODUCTION_SIZE;
    if (production_queue.count < PRODUCTION_SIZE)
        production_queue.count++;
}

void insert_into_filtered_queue(sensor_data data) {
    size_t raw_size = sizeof(filtered_queue.buffer[filtered_queue.head].raw) / sizeof(filtered_queue.buffer[filtered_queue.head].raw[0]);

    for (size_t i = 0; i < raw_size; ++i)
        filtered_queue.buffer[filtered_queue.head].raw[i] = data.raw[i];

    filtered_queue.head = (filtered_queue.head + 1) % FILTERED_SIZE;
    if (filtered_queue.count < FILTERED_SIZE)
        filtered_queue.count++;
}


void sort_queue(void *queue_ptr) {
    prod_queue *queue = (prod_queue *)queue_ptr; // cast to either queues; doesn't matter as they only differ in buffer size

    size_t size = sizeof(queue->buffer[queue->head].raw) / sizeof(queue->buffer[queue->head].raw[0]);
    for (size_t i = 0; i < size; ++i) { // loop through all data streams (e.g., ax, ay, ...)
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
        
        sort_queue(&sorted_queue);

        xSemaphoreTake(mutex_2, portMAX_DELAY);
        insert_into_filtered_queue(sorted_queue.buffer[sorted_queue.count / 2]);
        xSemaphoreGive(mutex_2);
        
        xSemaphoreGive(processing_done);
    }
}