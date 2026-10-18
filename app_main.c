#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "mpu_6050.h"

static const char *TAG = "APP";

#define I2C_PORT_NUM    I2C_NUM_0
#define I2C_SDA_PIN     GPIO_NUM_21
#define I2C_SCL_PIN     GPIO_NUM_22

void app_main(void)
{
    /* 1. Configure and install the I2C Master Bus */
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT_NUM,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    /* 2. Attach MPU6050 as a device on the bus */
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_I2C_ADDR,
        .scl_speed_hz = 400000, /* 400 kHz Fast Mode */
    };

    i2c_master_dev_handle_t mpu6050_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &mpu6050_handle));

    /* 3. Initialize MPU6050 registers */
    ESP_ERROR_CHECK(mpu6050_init(mpu6050_handle));

    mpu6050_raw_data_t raw;
    mpu6050_scaled_data_t scaled;

    /* 4. Sensor Acquisition Loop */
    while (1) {
        esp_err_t err = mpu6050_read_raw(mpu6050_handle, &raw);
        if (err == ESP_OK) {
            mpu6050_convert_units(&raw, &scaled);
            ESP_LOGI(TAG, "Accel: [%.2f, %.2f, %.2f] g | Gyro: [%.1f, %.1f, %.1f] dps | Temp: %.1f C",
                     scaled.accel_x_g, scaled.accel_y_g, scaled.accel_z_g,
                     scaled.gyro_x_dps, scaled.gyro_y_dps, scaled.gyro_z_dps,
                     scaled.temp_c);
        } else {
            ESP_LOGW(TAG, "I2C Transaction Failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(100)); /* 10 Hz telemetry */
    }
}