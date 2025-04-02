#include "sensor_init.h"

sensor_t *sensors[] = {
    &(sensor_t){ SENSOR_MPU9250, MPU9250_ADDR, 0 },
    &(sensor_t){ SENSOR_BMP280, BMP280_ADDR, 0 }
};

void app_main(void) {    
    
    xTaskCreatePinnedToCore(i2c_sensor_read, "SensorCollection", 2048, sensors, 3, NULL, tskNO_AFFINITY);
    
    vTaskDelete(NULL);
}
