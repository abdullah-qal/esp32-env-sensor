#include "sensor_init.h"

const char *I2C_TAG = "I2C_INIT";
const char *SENSOR_TAG = "SENSOR_INIT";

esp_err_t register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

esp_err_t register_write(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

esp_err_t i2c_master_init(i2c_master_bus_handle_t *bus_handle) {
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&bus_config, bus_handle);
}

esp_err_t i2c_dev_init(i2c_master_bus_handle_t bus_handle, sensor_t *sensor) {
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = sensor->addr,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_config, &sensor->dev_handle);
    if (err != ESP_OK) 
        ESP_LOGE(I2C_TAG, "Failed to create I2C device at address 0x%02X.", sensor->addr);
    return err;
}

esp_err_t mpu9250_init(sensor_t *sensor) {
    uint8_t contents[2];
    esp_err_t err = register_read(sensor->dev_handle, MPU9250_GYRO_CONFIG, contents, sizeof(contents));
    
    if (err != ESP_OK) {
        ESP_LOGE(SENSOR_TAG, "Failed to initialise the MPU9250 sensor.");
        return err;
    }

    uint8_t mask = 0x18;
    for (int i = 0; i < 2; ++i) {
        contents[i] = (contents[i] & ~mask) | ((i != 1 ? MPU9250_GYRO_MODE : MPU9250_ACCEL_MODE) << 3);
        register_write(sensor->dev_handle, MPU9250_GYRO_CONFIG + i, contents[i]);
    }
    return ESP_OK;
}

float mpu9250_compensate_A_f(sensor_t *sensor, int16_t adc_A) {
    uint8_t contents;
    if (register_read(sensor->dev_handle, MPU9250_ACCEL_CONFIG, &contents, 1) != ESP_OK) {
        ESP_LOGE(SENSOR_TAG, "Failed to read accelerometer config.");
        return 0;
    }
    switch (contents) {
        case 0x00: return adc_A / 16384.0f;
        case 0x01: return adc_A / 8192.0f;
        case 0x10: return adc_A / 4096.0f;
        case 0x11: return adc_A / 2048.0f;
        default:
            ESP_LOGE(SENSOR_TAG, "Unexpected accelerometer config value: 0x%02X", contents);
            return 0;
    }
}

float mpu9250_compensate_G_f(sensor_t *sensor, int16_t adc_G) {
    uint8_t contents;
    if (register_read(sensor->dev_handle, MPU9250_GYRO_CONFIG, &contents, 1) != ESP_OK) {
        ESP_LOGE(SENSOR_TAG, "Failed to read gyroscope config.");
        return 0;
    }
    switch (contents) {
        case 0x00: return adc_G / 131.0f;
        case 0x01: return adc_G / 65.5f;
        case 0x10: return adc_G / 32.8f;
        case 0x11: return adc_G / 16.4f;
        default:
            ESP_LOGE(SENSOR_TAG, "Unexpected gyroscope config value: 0x%02X.", contents);
            return 0;
    }
}
int32_t bmp280_compensate_T_int32(int32_t adc_T, struct bmp280_data *data) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)data->T1 << 1))) * ((int32_t)data->T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)data->T1)) * ((adc_T >> 4) - ((int32_t)data->T1))) >> 12) *
           ((int32_t)data->T3)) >> 14;
    data->t_fine = var1 + var2;
    T = (data->t_fine * 5 + 128) >> 8;
    return T;
}

uint32_t bmp280_compensate_P_uint32(int32_t adc_P, struct bmp280_data *data) {
    int64_t var1, var2, p;
    var1 = ((int64_t)data->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)data->P6;
    var2 = var2 + ((var1 * (int64_t)data->P5) << 17);
    var2 = var2 + (((int64_t)data->P4) << 35);
    var1 = ((var1 * var1 * (int64_t)data->P3) >> 8) + ((var1 * (int64_t)data->P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)data->P1) >> 33;
    if (var1 == 0) 
        return 0; // avoid exception caused by division by zero

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)data->P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)data->P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)data->P7) << 4);
    return (uint32_t)p; 
}

struct bmp280_data bmp280_read_calibration_data(sensor_t *sensor) {
    uint8_t calib_data[24];
    struct bmp280_data dig = {0};

    if (register_read(sensor->dev_handle, BMP280_DIG_REG_START, calib_data, sizeof(calib_data)) != ESP_OK) {
        ESP_LOGE(SENSOR_TAG, "Failed to read calibration data.");
        return dig;
    } 

    int i = 0;
    for (int j = 0; j < 3; ++i, ++j) 
        dig.Ts[j] = (calib_data[2 * i + 1] << 8) | calib_data[2 * i]; 
    for (int j = 0; j < 9; ++i, ++j) 
        dig.Ps[j] = (calib_data[2 * i + 1] << 8) | calib_data[2 * i]; 
    return dig;
}

esp_err_t bmp280_init(sensor_t *sensor) {
    uint8_t control_value = BMP280_MODE_NORMAL | (BMP280_OSRS_T << 5) | (BMP280_OSRS_P << 2);
    esp_err_t err = register_write(sensor->dev_handle, BMP280_REG_CONTROL, control_value);
    
    if (err != ESP_OK) 
        ESP_LOGE(SENSOR_TAG, "Failed to initialize BMP280.");

    return err;
}

void i2c_sensor_read(void *pvParameters) {
    sensor_t **sensors = (sensor_t **)pvParameters; 
    uint8_t data[14];  // Buffer for storing sensor data

    i2c_master_bus_handle_t bus_handle = NULL;

    if (i2c_master_init(&bus_handle) != ESP_OK) {
        ESP_LOGE(I2C_TAG, "Failed to create I2C Bus.");
        vTaskSuspend(NULL);
    }

    for (int i = 0; i < sizeof(sensors) / sizeof(sensors[0]); i++) {
        if (i2c_dev_init(bus_handle, sensors[i]) != ESP_OK) {
            ESP_LOGE(I2C_TAG, "Failed to create I2C device at address 0x%02X.", sensors[i]->addr);
            vTaskSuspend(NULL);
        }
        ESP_LOGI(I2C_TAG, "Initialized sensor at address 0x%02X", sensors[i]->addr);

        if (sensors[i]->name == SENSOR_BMP280) bmp280_init(sensors[i]);
        if (sensors[i]->name == SENSOR_MPU9250) mpu9250_init(sensors[i]);
    }
    while (1) {
        for (int i = 0; i < sizeof(sensors) / sizeof(sensors[0]); i++) {
            switch (sensors[i]->name) {
                case SENSOR_MPU9250: {
                    // Read data from MPU9250 sensor
                    if (register_read(sensors[i]->dev_handle, MPU9250_ACCEL_GYRO_REG_START, data, 14) != ESP_OK) {
                        ESP_LOGE(SENSOR_TAG, "Failed to read MPU9250.");
                        continue;
                    }

                    int16_t mpu9250_values[7];
                    float normalized_values[7];

                    for (int j = 0; j < 7; ++j) {
                        mpu9250_values[j] = (int16_t)((data[2 * j] << 8) | data[2 * j + 1]);
                        normalized_values[j] = (j > 3) ? mpu9250_compensate_G_f(sensors[i], mpu9250_values[j])
                                                       : mpu9250_compensate_A_f(sensors[i], mpu9250_values[j]);
                    }

                    ESP_LOGI(SENSOR_TAG, "========== SENSOR DATA ==========");
                    ESP_LOGI(SENSOR_TAG, "MPU9250 | Accel (g):  X=%.2f   Y=%.2f  Z=%.2f", 
                            normalized_values[0], normalized_values[1], normalized_values[2]);
                    ESP_LOGI(SENSOR_TAG, "        | Gyro (°/s): X=%.2f  Y=%.2f  Z=%.2f", 
                            normalized_values[4], normalized_values[5], normalized_values[6]);
                } break;

                case SENSOR_BMP280: {
                    // Read data from BMP280 sensor
                    struct bmp280_data b_data = bmp280_read_calibration_data(sensors[i]);

                    if (register_read(sensors[i]->dev_handle, BMP280_REG_START, data, 6) != ESP_OK) {
                        ESP_LOGE(SENSOR_TAG, "Failed to read BMP280.");
                        continue; 
                    }

                    int32_t raw_pressure = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
                    int32_t raw_temperature = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);

                    int32_t temperature = bmp280_compensate_T_int32(raw_temperature, &b_data);
                    uint32_t pressure = bmp280_compensate_P_uint32(raw_pressure, &b_data);

                    ESP_LOGI(SENSOR_TAG, "BMP280  | Temp (°C): %.2f  |  Pressure (hPa): %.2f", 
                            temperature / 100.0f, pressure / 25600.0f);
                    ESP_LOGI(SENSOR_TAG, "=================================\n");
                } break;

                default:
                    ESP_LOGW(SENSOR_TAG, "Unknown sensor type.");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
