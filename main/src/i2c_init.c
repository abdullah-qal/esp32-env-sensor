#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2c_init.h"

static const char *I2C_TAG = "I2C_INIT";

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
    esp_err_t err = i2c_new_master_bus(&bus_config, bus_handle);
    if (err != ESP_OK)
        ESP_LOGE(I2C_TAG, "Failed to create I2C Bus.");
    return err;

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
    ESP_LOGI(I2C_TAG, "Initialized sensor at address 0x%02X", sensor->addr);
    return err;
}
