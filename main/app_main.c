#include "sensor_init.h"


void app_main(void) {    

    xTaskCreatePinnedToCore(i2c_sensor_read, "SensorCollection", 2048, NULL, 3, NULL, tskNO_AFFINITY);
    
    vTaskDelete(NULL);
}
