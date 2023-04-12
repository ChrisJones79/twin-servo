#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "bmp280.h"



// LEFT-OFF: the bmp2_dev doesn't have a member for its address so
// hard-code it in to the read command above. This command will only 
// ever be used to read the I2C bus address for the bmp280.
// Just follow the cmd_link pattern.

struct bmp2_dev bmp = {
    .chip_id = BMP2_CHIP_ID,
    .intf = BMP2_I2C_INTF,
    .read = bmp2_read,
    .write = bmp2_write,
    .delay_us = bmp2_delay_us,
    .intf_ptr = BMP2_I2C_INTF,
    .power_mode = BMP2_SLEEP_MODE,
    .calib_param = &bmp2_calib_param
};


void app_main(void){
    ESP_LOGI("main", "Entering main.");
    ESP_LOGI("main", "Initializing BMP280");

    // bmp.chip_id = BMP2_CHIP_ID;
    // bmp.intf = BMP2_I2C_INTF;

}