#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "bmp280.h"



// LEFT-OFF: the bmp2_dev doesn't have a member for its address so
// hard-code it in to the read command above. This command will only 
// ever be used to read the I2C bus address for the bmp280.
// Just follow the cmd_link pattern.

struct bmp2_dev dev;

static int8_t get_data(uint32_t period, struct bmp2_config *conf, struct bmp2_dev *dev);
int8_t bmp2_interface_selection(struct bmp2_dev *dev, uint8_t intf);


void app_main(void){
    ESP_LOGI("main", "Entering main.");
    ESP_LOGI("main", "Initializing BMP280");

    int8_t rslt;
    uint32_t meas_time;
    struct bmp2_config conf;

    /* Interface selection is to be updated as parameter
     * For I2C :  BMP2_I2C_INTF
     * For SPI :  BMP2_SPI_INTF
     */
    rslt = bmp2_interface_selection(&dev, BMP2_I2C_INTF);
    bmp2_error_codes_print_result("bmp2_interface_selection", rslt);

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


esp_err_t bmp_init(struct bmp2_dev *dev, uint8_t intf){
    esp_err_t ret = init_i2c();
    if (ret != ESP_OK)
    {
        ESP_LOGE("I2C_init", "failed: %s", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI("I2C_init", "success");
    }
    return ret;
}

int8_t bmp2_interface_selection(struct bmp2_dev *dev, uint8_t intf)
{
    int8_t rslt = BMP2_OK;

    if (dev != NULL)
    {
        int16_t result = coines_open_comm_intf(COINES_COMM_INTF_USB);

        if (result < COINES_SUCCESS)
        {
            printf(
                "\n Unable to connect with Application Board ! \n" " 1. Check if the board is connected and powered on. \n" " 2. Check if Application Board USB driver is installed. \n"
                " 3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
            exit(result);
        }

        coines_set_shuttleboard_vdd_vddio_config(0, 0);
        coines_delay_msec(100);

        /* Bus configuration : I2C */
        if (intf == BMP2_I2C_INTF)
        {
            printf("I2C Interface\n");

            dev_addr = BMP2_I2C_ADDR_PRIM;
            dev->read = bmp2_i2c_read;
            dev->write = bmp2_i2c_write;
            dev->intf = BMP2_I2C_INTF;

            coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);
        }
        /* Bus configuration : SPI */
        else if (intf == BMP2_SPI_INTF)
        {
            printf("SPI Interface\n");

            dev_addr = COINES_SHUTTLE_PIN_7;
            dev->read = bmp2_spi_read;
            dev->write = bmp2_spi_write;
            dev->intf = BMP2_SPI_INTF;

            coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_7_5_MHZ, COINES_SPI_MODE0);
        }

        /* Holds the I2C device addr or SPI chip selection */
        dev->intf_ptr = &dev_addr;

        /* Configure delay in microseconds */
        dev->delay_us = bmp2_delay_us;

        coines_delay_msec(100);

        coines_set_shuttleboard_vdd_vddio_config(3300, 3300);

        coines_delay_msec(100);
    }
    else
    {
        rslt = BMP2_E_NULL_PTR;
    }

    return rslt;
}

static int8_t get_data(uint32_t period, struct bmp2_config *conf, struct bmp2_dev *dev)
{
    int8_t rslt = BMP2_E_NULL_PTR;
    int8_t idx = 1;
    struct bmp2_status status;
    struct bmp2_data comp_data;

    printf("Measurement delay : %lu us\n", (long unsigned int)period);

    while (idx <= 50)
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