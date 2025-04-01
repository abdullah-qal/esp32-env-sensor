#ifndef SENSOR_H
#define SENSOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TIMEOUT_MS 1000

// I2C address for each of the sensors
#define BMP280_ADDR 0x77 
#define MPU9250_ADDR 0x68

// Starting (sensor) registers for each of the sensors
#define BMP280_REG_START 0xF7 
#define MPU9250_ACCEL_GYRO_REG_START 0x3B

// Config and control registers for each of the registers
#define BMP280_REG_CONTROL 0xF4  // Control register 
#define BMP280_REG_CONFIG 0xF5   // Configuration register 
#define BMP280_DIG_REG_START 0x88 // For value trimming

#define MPU9250_GYRO_CONFIG 0x1B // For GYRO_MODE
#define MPU9250_ACCEL_CONFIG 0x1C // For ACCEL_MODE

#define MPU9250_GYRO_MODE 0x00 // 0x00: +250dps, 0x01: +500dps, 0x10: +1000dps, 0x11: +2000dps
#define MPU9250_ACCEL_MODE 0x00 // 0x00: \pm 2g, 0x01: \pm 4g, 0x10: \pm 8g, 0x11: \pm 16g

// BMP280 Mode definitions
#define BMP280_MODE_SLEEP 0x00
#define BMP280_MODE_FORCED 0x01
#define BMP280_MODE_NORMAL 0x03

// Oversampling settings 
#define BMP280_OSRS_T 0x01  // Temperature oversampling
#define BMP280_OSRS_P 0x01  // Pressure oversampling

extern const char *I2C_TAG;
extern const char *SENSOR_TAG;

typedef enum {
    SENSOR_MPU9250,
    SENSOR_BMP280,
} sensor_name_t;

typedef struct {
    sensor_name_t const name;
    uint8_t const addr;
    i2c_master_dev_handle_t dev_handle;
} sensor_t;

esp_err_t register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t register_write(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data);

esp_err_t i2c_master_init(i2c_master_bus_handle_t *bus_handle);
esp_err_t i2c_dev_init(i2c_master_bus_handle_t bus_handle, sensor_t *sensor);

// BMP280-specific functions mostly taken from the datasheet
struct bmp280_data {
    union {
        uint16_t Ts[3];
        struct {
            uint16_t T1, T2, T3;
        };
    };
    union {
        uint16_t Ps[9];
        struct {
            uint16_t P1, P2, P3, P4, P5, P6, P7, P8, P9;
        };
    };
    int32_t t_fine;
};

esp_err_t bmp280_init(sensor_t *sensor);

struct bmp280_data bmp280_read_calibration_data(sensor_t *sensor);

int32_t bmp280_compensate_T_int32(int32_t adc_T, struct bmp280_data *data);
uint32_t bmp280_compensate_P_uint32(int32_t adc_P, struct bmp280_data *data);

// MPU9250-specific functions 
esp_err_t mpu9250_init(sensor_t *sensor);

float mpu9250_compensate_A_f(sensor_t *sensor, int16_t adc_A);
float mpu9250_compensate_G_f(sensor_t *sensor, int16_t adc_A);

void i2c_sensor_read(void *pvParameters);

#endif