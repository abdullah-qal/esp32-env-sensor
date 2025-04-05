#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "include/sensor_read.h"
#include "include/sensor_process.h"
#include "include/sensor_transmit.h"

void app_main(void) {        
    mutex_1 = xSemaphoreCreateMutex();
    mutex_2 = xSemaphoreCreateMutex();
    processing_done = xSemaphoreCreateBinary();
    reading_done = xSemaphoreCreateBinary();

    // Check if creation succeeded
    if (!mutex_1 || !mutex_2 || !processing_done || !reading_done) {
        ESP_LOGE("INIT", "Semaphore creation failed");
        abort();
    }

    i2c_sensor_init();

    xTaskCreatePinnedToCore(i2c_sensor_read, "sensor_collection", 4096, NULL, 3, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(filter_sensor_value, "sensor_processing", 4096, NULL, 2, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(print_sensor_value, "printing_data", 4096, NULL, 1  , NULL, tskNO_AFFINITY);
    
    vTaskDelete(NULL);
}
