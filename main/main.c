#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "bmp2.h"

// LEFT-OFF: the bmp2_dev doesn't have a member for its address so
// hard-code it in to the read command above. This command will only
// ever be used to read the I2C bus address for the bmp280.
// Just follow the cmd_link pattern.


static int8_t get_data(uint32_t period, struct bmp2_config *conf, struct bmp2_dev *dev);

void app_main(void)
{

    ESP_LOGI("main", "Entering main.");
    ESP_LOGI("main", "Initializing I2C bus.");
    init_i2c(); 
    ESP_LOGI("main", "Initializing BMP280");

    int8_t rslt;
    uint32_t meas_time;
    struct bmp2_config conf;
    struct bmp2_dev dev;


    rslt = bmp2_interface_selection(&dev, BMP2_I2C_INTF);
    bmp2_error_codes_print_result("bmp2_interface_selection", rslt);

    // uint8_t res[4];
    // esp_err_t ret = ESP_OK;

    // // We will loop through the registers reading four bytes each time
    // // and use printf to print the results.
    // for(int i = 0; i < 256; i++)
    // {
    //     ret = bmp2_read(i, res, 4, &dev);
    //     printf("0x%02x: 0x%02x 0x%02x 0x%02x 0x%02x\n", i, res[0], res[1], res[2], res[3]);
    //     if(ret != ESP_OK)
    //     {
    //         printf(" - ERROR: %s", esp_err_to_name(ret));
    //     }
    //     vTaskDelay(100 / portTICK_PERIOD_MS);
    // }

    rslt = bmp2_init(&dev);
    bmp2_error_codes_print_result("bmp2_init", rslt);

    /* Always read the current settings before writing, especially when all the configuration is not modified */
    rslt = bmp2_get_config(&conf, &dev);
    bmp2_error_codes_print_result("bmp2_get_config", rslt);

    /* Configuring the over-sampling mode, filter coefficient and output data rate */
    /* Overwrite the desired settings */
    conf.filter = BMP2_FILTER_OFF;

    /* Over-sampling mode is set as high resolution i.e., os_pres = 8x and os_temp = 1x */
    conf.os_mode = BMP2_OS_MODE_HIGH_RESOLUTION;

    /* Setting the output data rate */
    conf.odr = BMP2_ODR_250_MS;

    rslt = bmp2_set_config(&conf, &dev);
    bmp2_error_codes_print_result("bmp2_set_config", rslt);

    /* Set normal power mode */
    rslt = bmp2_set_power_mode(BMP2_POWERMODE_NORMAL, &conf, &dev);
    bmp2_error_codes_print_result("bmp2_set_power_mode", rslt);

    /* Calculate measurement time in microseconds */
    rslt = bmp2_compute_meas_time(&meas_time, &conf, &dev);
    bmp2_error_codes_print_result("bmp2_compute_meas_time", rslt);

    /* Read pressure and temperature data */
    rslt = get_data(meas_time, &conf, &dev);

    // bmp.chip_id = BMP2_CHIP_ID;
    // bmp.intf = BMP2_I2C_INTF;
}


static int8_t get_data(uint32_t period, struct bmp2_config *conf, struct bmp2_dev *dev)
{
    int8_t rslt = BMP2_E_NULL_PTR;
    int8_t idx = 1;
    struct bmp2_status status;
    struct bmp2_data comp_data;

    printf("Measurement delay : %lu us\n", (long unsigned int)period);

    while (idx <= 5000)
    {
        rslt = bmp2_get_status(&status, dev);
        bmp2_error_codes_print_result("bmp2_get_status", rslt);

        if (status.measuring == BMP2_MEAS_DONE)
        {
            /* Delay between measurements */
            dev->delay_us(period, dev->intf_ptr);

            /* Read compensated data */
            rslt = bmp2_get_sensor_data(&comp_data, dev);
            bmp2_error_codes_print_result("bmp2_get_sensor_data", rslt);

#ifdef BMP2_64BIT_COMPENSATION
            comp_data.pressure = comp_data.pressure / 256;
#endif

#ifdef BMP2_DOUBLE_COMPENSATION
            printf("Data[%d]:    Temperature: %.4lf deg C	Pressure: %.4lf Pa\n",
                   idx,
                   comp_data.temperature,
                   comp_data.pressure);
#else
            printf("Data[%d]:    Temperature: %ld deg C	Pressure: %lu Pa\n", idx, (long int)comp_data.temperature,
                   (long unsigned int)comp_data.pressure);
#endif

            idx++;
        }
    }

    return rslt;
}