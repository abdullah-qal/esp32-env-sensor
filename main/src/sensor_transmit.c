#include "esp_log.h"

#include "include/sensor_transmit.h"
#include "include/sensor_process.h"

static const char *SENSOR_TAG = "SENSOR_TRANSMISSION";

void print_sensor_value(void *pvParameters) {
    while (1) {
        xSemaphoreTake(processing_done, portMAX_DELAY);
        xSemaphoreTake(mutex_2, portMAX_DELAY);

        sensor_data data = filtered_queue.buffer[(filtered_queue.head  + FILTERED_SIZE - 1) % FILTERED_SIZE];
        ESP_LOGI(SENSOR_TAG, 
            "========== SENSOR DATA ==========");
            ESP_LOGI(SENSOR_TAG, "MPU9250 | Accel (g):  X=%d.%02d   Y=%d.%02d  Z=%d.%02d",
                (int16_t)data.ax / SCALE_MULTIPLIER, data.ax % SCALE_MULTIPLIER,
                (int16_t)data.ay / SCALE_MULTIPLIER, data.ay % SCALE_MULTIPLIER,
                (int16_t)data.az / SCALE_MULTIPLIER, data.az % SCALE_MULTIPLIER);
            
            
            ESP_LOGI(SENSOR_TAG, "        | Gyro (°/s): X=%d.%02d  Y=%d.%02d  Z=%d.%02d", 
                (int16_t)data.gx / SCALE_MULTIPLIER, data.gx % SCALE_MULTIPLIER,
                (int16_t)data.gy / SCALE_MULTIPLIER, data.gy % SCALE_MULTIPLIER,
                (int16_t)data.gz / SCALE_MULTIPLIER, data.gz % SCALE_MULTIPLIER);
                
            ESP_LOGI(SENSOR_TAG, 
                "BMP280  | Temp (°C): %d.%02d  |  Pressure (hPa): %d.%02d", 
                (int32_t)data.temp / SCALE_MULTIPLIER, data.temp % SCALE_MULTIPLIER,  
                          data.pres / SCALE_MULTIPLIER, data.pres % SCALE_MULTIPLIER);
            
        ESP_LOGI(SENSOR_TAG, 
            "=================================\n");

        xSemaphoreGive(mutex_2);
    }
}