#include "sensor_init.h"

void app_main(void) {
    i2c_master_bus_handle_t bus_handle = NULL;
    if (i2c_master_init(&bus_handle) != ESP_OK) {
        ESP_LOGE(I2C_TAG, "Failed to create I2C Bus.");
        return;
    }

    sensor_t sensors[] = {
        { SENSOR_MPU9250, MPU9250_ADDR, 0 },
        { SENSOR_BMP280, BMP280_ADDR, 0 }
    };

    for (int i = 0; i < sizeof(sensors) / sizeof(sensors[0]); i++) {
        if (i2c_dev_init(bus_handle, &sensors[i]) == ESP_OK) {
            ESP_LOGI(I2C_TAG, "Initialized sensor at address 0x%02X", sensors[i].addr);
            if (sensors[i].name == SENSOR_BMP280) bmp280_init(&sensors[i]);
            if (sensors[i].name == SENSOR_MPU9250) mpu9250_init(&sensors[i]);

            xTaskCreatePinnedToCore(i2c_sensor_read, "SensorTask", 4096, &sensors[i], 3, NULL, tskNO_AFFINITY);
        }
    }
    while (1) 
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1-second delay to keep app_main from exiting
}
