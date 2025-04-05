#ifndef I2C_INIT_H
#define I2C_INIT_H

#include "driver/i2c_master.h"
#include "esp_log.h"

#include "sensor_config.h"

// I2C Bus Configurations
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TIMEOUT_MS 1000

esp_err_t register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t register_write(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data);

esp_err_t i2c_master_init(i2c_master_bus_handle_t *bus_handle);
esp_err_t i2c_dev_init(i2c_master_bus_handle_t bus_handle, sensor_t *sensor);

#endif