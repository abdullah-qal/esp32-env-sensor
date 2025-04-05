#include "include/sensor_config.h"
#include "include/i2c_init.h"

static const char *CONFIG_TAG = "SENSOR_CONFIG";

esp_err_t bmp280_init(sensor_t *sensor) {
    uint8_t control_value = BMP280_MODE_NORMAL | (BMP280_OSRS_T << 5) | (BMP280_OSRS_P << 2);
    esp_err_t err = register_write(sensor->dev_handle, BMP280_REG_CONTROL, control_value);
    
    if (err != ESP_OK) 
        ESP_LOGE(CONFIG_TAG, "Failed to initialize BMP280.");

    return err;
}

struct bmp280_data bmp280_read_calibration_data(sensor_t *sensor) {
    uint8_t calib_data[24];
    struct bmp280_data dig = {0};

    if (register_read(sensor->dev_handle, BMP280_DIG_REG_START, calib_data, sizeof(calib_data)) != ESP_OK) {
        ESP_LOGE(CONFIG_TAG, "Failed to read calibration data.");
        return dig;
    } 

    int i = 0;
    for (size_t j = 0; j < 3; ++i, ++j) 
        dig.Ts[j] = (calib_data[2 * i + 1] << 8) | calib_data[2 * i]; 
    for (size_t j = 0; j < 9; ++i, ++j) 
        dig.Ps[j] = (calib_data[2 * i + 1] << 8) | calib_data[2 * i]; 
    return dig;
}

uint32_t bmp280_compensate_T_uint32(int32_t adc_T, struct bmp280_data *data) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)data->T1 << 1))) * ((int32_t)data->T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)data->T1)) * ((adc_T >> 4) - ((int32_t)data->T1))) >> 12) *
           ((int32_t)data->T3)) >> 14;
    data->t_fine = var1 + var2;
    T = (data->t_fine * 5 + 128) >> 8;
    return (uint32_t)((SCALE_MULTIPLIER * T) / 100);
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
    return (uint32_t)((SCALE_MULTIPLIER * p) / 25600); 
}

esp_err_t mpu9250_init(sensor_t *sensor) {
    uint8_t contents[2];
    esp_err_t err = register_read(sensor->dev_handle, MPU9250_GYRO_CONFIG, contents, sizeof(contents));
    
    if (err != ESP_OK) {
        ESP_LOGE(CONFIG_TAG, "Failed to initialise the MPU9250 sensor.");
        return err;
    }

    uint8_t mask = 0x18;
    for (size_t i = 0; i < 2; ++i) {
        contents[i] = (contents[i] & ~mask) | ((i != 1 ? MPU9250_GYRO_MODE : MPU9250_ACCEL_MODE) << 3);
        register_write(sensor->dev_handle, MPU9250_GYRO_CONFIG + i, contents[i]);
    }
    return ESP_OK;
}

uint32_t mpu9250_compensate_A_uint32(sensor_t *sensor) {
    uint8_t contents;
    if (register_read(sensor->dev_handle, MPU9250_ACCEL_CONFIG, &contents, 1) != ESP_OK) {
        ESP_LOGE(CONFIG_TAG, "Failed to read accelerometer config.");
        return 0;
    }
    switch (contents) {
        case 0x00: return 16384; //adc_A/16384 -> 10 * adc_A/1310 -> 1000 * (blah) / 256000.0f -> blah / 256 
        case 0x01: return 8192;
        case 0x10: return 4096;
        case 0x11: return 2048;
        default:
            ESP_LOGE(CONFIG_TAG, "Unexpected accelerometer config value: 0x%02X", contents);
            return 0;
    }
}

uint32_t mpu9250_compensate_G_uint32(sensor_t *sensor) {
    uint8_t contents;
    if (register_read(sensor->dev_handle, MPU9250_GYRO_CONFIG, &contents, 1) != ESP_OK) {
        ESP_LOGE(CONFIG_TAG, "Failed to read gyroscope config.");
        return 0;
    }
    switch (contents) {
        case 0x00: return 1310; //adc_A/131.0 -> 10 * adc_A/1310 -> 1000 * (blah) -> divide by 
        case 0x01: return 655;
        case 0x10: return 328;
        case 0x11: return 164;
        default:
            ESP_LOGE(CONFIG_TAG, "Unexpected gyroscope config value: 0x%02X.", contents);
            return 0;
    }
}