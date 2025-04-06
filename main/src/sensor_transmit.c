#include "esp_log.h"

#include "sensor_transmit.h"

static const char *TAG = "SENSOR_TRANSMIT";

uint32_t isqrt(uint32_t x) {
    uint32_t res = 0;
    uint32_t bit = 1UL << 30;

    while (bit > x) bit >>= 2;

    while (bit != 0) {
        if (x >= res + bit) {
            x -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }

    return res;
}

ble_data calculate_data(fil_queue q) {
    ble_data output = {0};

    sort_queue((queue *)&q);

    output.min = q.buffer[0];
    output.max = q.buffer[q.count - 1];
    output.median = q.buffer[q.count / 2];

    size_t raw_size = sizeof(q.buffer[q.head].raw) / sizeof(q.buffer[q.head].raw[0]);

    for (size_t i = 0; i < raw_size; ++i) { // Loop through all data streams (e.g., ax, ay, ...)
        int32_t mean = 0;
        int64_t M2 = 0;
        for (size_t j = 0; j < q.count; ++j) {
            int32_t x = (int32_t)q.buffer[j].raw[i];
            int32_t delta = x - mean;
            mean += delta / (j + 1);
            int32_t delta2 = x- mean;
            M2 += (int64_t)delta * delta2;
        }
        output.stddev.raw[i] = isqrt((uint32_t)(M2 / (q.count - 1)));
    }
    return output;
}


void print_sensor_value(void *pvParameters) {
    while (1) {
        if (filtered_queue.count < 2) 
            continue;

        xSemaphoreTake(processing_done, portMAX_DELAY);
        xSemaphoreTake(mutex_2, portMAX_DELAY);
        ble_data result = calculate_data(filtered_queue);

        ESP_LOGI(TAG, "Raw[0] values in queue:");
        for (size_t i = 0; i < filtered_queue.count; ++i) 
            ESP_LOGI(TAG, "  [%02d] = %d", i, filtered_queue.buffer[i].raw[0]);
    
        ESP_LOGI(TAG, "------ Summary (raw[0]) ------");
        ESP_LOGI(TAG, "Min     : %d", result.min.raw[0]);
        ESP_LOGI(TAG, "Max     : %d", result.max.raw[0]);
        ESP_LOGI(TAG, "Median  : %d", result.median.raw[0]);
        ESP_LOGI(TAG, "Stddev  : %d", result.stddev.raw[0]);

        xSemaphoreGive(mutex_2);
    }
}