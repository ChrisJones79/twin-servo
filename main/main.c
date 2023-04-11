#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "bmp280.h"

esp_err_t bmp2_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct bmp2_dev *dev){
    esp_err_t rslt = ESP_FAIL;
    
    // i2c_cmd_handle_t cmd = 
}

// LEFT-OFF: the bmp2_dev doesn't have a member for its address so
// hard-code it in to the read command above. This command will only 
// ever be used to read the I2C bus address for the bmp280.
// Just follow the cmd_link pattern.

bmp2_dev bmp = {
    .chip_id = BMP2_CHIP_ID,
    .intf = BMP2_I2C_INTF,
}


void app_main(void){
    ESP_LOGI("main", "Entering main.");
    ESP_LOGI("main", "Initializing BMP280");

    bmp.chip_id = BMP2_CHIP_ID;
    bmp.intf = BMP2_I2C_INTF

}