#include "driver/i2c_master.h" 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static char const *TAG = "I2C_Initialisation";

#define I2C_MASTER_SCL_IO 22    // Defines SCL pin
#define I2C_MASTER_SDA_IO 21    // Defines SDA pin
#define I2C_MASTER_NUM I2C_NUM_0 // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ 100000 // 100 kHz SCL
#define I2C_MASTER_TX_BUF_DISABLE 0 // I2C master doesn't need buffer 
#define I2C_MASTER_RX_BUF_DISABLE 0 // I2C master doesn't need buffer 
#define I2C_MASTER_TIMEOUT_MS 1000 // Timeout incase of a hang


static esp_err_t register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle, uint8_t addr) {

    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT, 
        .sda_io_num = I2C_MASTER_SDA_IO,                   
        .scl_io_num = I2C_MASTER_SCL_IO,                   
        .glitch_ignore_cnt = 7,             
        .flags.enable_internal_pullup = true, 
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus");
        return err;
    }
    return ESP_OK;
}

void app_main(void) {    
    uint8_t data[2];
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    esp_err_t err;
    uint8_t addrs[] = {0x68, 0x77}; // 0x68 for MPU9250, 0x77 for BMP280 
    uint8_t *last_addr = addrs + sizeof(addrs) / size(addrs[0]);

    ESP_LOGI(TAG, "I2C initialization starting...");

    for (uint8_t *addr = addrs; addr < last_addr; ++addr) {
        ESP_LOGI(TAG, "Attempting to initialize device at address 0x%02X", *addr);

        err = i2c_master_init(&bus_handle, &dev_handle, *addr);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize device at address 0x%02X", *addr);
            continue;  // Try the next address
        }

        ESP_LOGI(TAG, "I2C device found at address 0x%02X", *addr);

        /* Cleanup I2C device */
        ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
        ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    } 

    ESP_LOGI(TAG, "I2C scan completed");
}
