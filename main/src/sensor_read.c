#include "esp_log.h"

#include "include/i2c_init.h"
#include "include/sensor_read.h"
#include "include/sensor_process.h"

static const char *SENSOR_TAG = "SENSOR_READ";

sensor_t sensors[] = {
    { SENSOR_MPU9250, MPU9250_ADDR, 0 },
    { SENSOR_BMP280, BMP280_ADDR, 0 }
};

SemaphoreHandle_t mutex_1;
SemaphoreHandle_t reading_done;

prod_queue production_queue = {0};

void i2c_sensor_init(void) {
    i2c_master_bus_handle_t bus_handle = NULL;
    if (i2c_master_init(&bus_handle) != ESP_OK) vTaskSuspend(NULL);

    for (size_t i = 0; i < sizeof(sensors) / sizeof(sensors[0]); ++i) {
        if (i2c_dev_init(bus_handle, &sensors[i]) != ESP_OK) vTaskSuspend(NULL);

        if (sensors[i].name == SENSOR_BMP280) bmp280_init(&sensors[i]);
        if (sensors[i].name == SENSOR_MPU9250) mpu9250_init(&sensors[i]);
    }
}

void i2c_sensor_read(void *pvParameters) {
    uint32_t accel_factor = SCALE_MULTIPLIER;
    uint32_t gyro_factor  = 10 * SCALE_MULTIPLIER;

    struct bmp280_data b_data = bmp280_read_calibration_data(&sensors[1]);
    while (1) {
        uint8_t data[14];  // Buffer for storing sensor data
        sensor_data collected_data = {0};
        size_t num_of_collected_data = 0;

        // Read data from MPU9250 sensor
        if (register_read(sensors[0].dev_handle, MPU9250_ACCEL_GYRO_REG_START, data, 14) != ESP_OK) {
            ESP_LOGE(SENSOR_TAG, "Failed to read MPU9250.");
            continue;
        }
        
        for (size_t j = 0; j < 7; ++j) {
            if (j == 3) continue;
            int16_t raw_value = ((int16_t)data[2 * j] << 8) | (int16_t)data[2 * j + 1];
            int32_t scale_factor = (j < 3) ? accel_factor : gyro_factor;
            int32_t compensation = (j < 3) ? mpu9250_compensate_A_uint32(&sensors[0]) 
                                            : mpu9250_compensate_G_uint32(&sensors[0]);
            int32_t scaled_value = (raw_value * scale_factor) / compensation;

            collected_data.raw[num_of_collected_data++] = (uint32_t)scaled_value;
        }

        // Read data from BMP280 sensor
        if (register_read(sensors[1].dev_handle, BMP280_REG_START, data, 6) != ESP_OK) {
            ESP_LOGE(SENSOR_TAG, "Failed to read BMP280.");
            continue; 
        }
        int32_t raw_pressure = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);
        int32_t raw_temp = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | ((int32_t)data[5] >> 4);

        collected_data.raw[num_of_collected_data++] = (uint32_t)(bmp280_compensate_P_uint32(raw_pressure, &b_data));
        collected_data.raw[num_of_collected_data++] = (uint32_t)(bmp280_compensate_T_uint32(raw_temp, &b_data));

        xSemaphoreTake(mutex_1, portMAX_DELAY);

        prod_enqueue(&production_queue, collected_data);

        xSemaphoreGive(mutex_1);
        xSemaphoreGive(reading_done);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
