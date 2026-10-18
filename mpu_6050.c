
#include "mpu_6050.h"
#include "esp_log.h"
#include <stddef.h>
#include <stdint.h>

static const char *TAG = "MPU6050";

#define I2C_TIMEOUT_MS 1000

/* Write one byte to an MPU6050 register */
static esp_err_t mpu6050_write_byte(
    i2c_master_dev_handle_t dev_handle,
    uint8_t reg,
    uint8_t data)
{
    uint8_t write_buffer[2] = {reg, data};

    return i2c_master_transmit(
        dev_handle,
        write_buffer,
        sizeof(write_buffer),
        I2C_TIMEOUT_MS
    );
}

/* Read multiple consecutive bytes starting from a register */
static esp_err_t mpu6050_read_bytes(
    i2c_master_dev_handle_t dev_handle,
    uint8_t reg,
    uint8_t *buffer,
    size_t len)
{
    return i2c_master_transmit_receive(
        dev_handle,
        &reg,
        1,
        buffer,
        len,
        I2C_TIMEOUT_MS
    );
}

/* Initialize and configure the MPU6050 */
esp_err_t mpu6050_init(i2c_master_dev_handle_t dev_handle)
{
    uint8_t who_am_i = 0;

    esp_err_t ret = mpu6050_read_bytes(
        dev_handle,
        MPU6050_REG_WHO_AM_I,
        &who_am_i,
        1
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I read failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    if (who_am_i != 0x68) {
        ESP_LOGE(TAG,
                 "Device identification failed: read 0x%02X, expected 0x68",
                 who_am_i);
        return ESP_ERR_NOT_FOUND;
    }

    /* Wake sensor and select the X-gyro PLL clock */
    ret = mpu6050_write_byte(
        dev_handle, MPU6050_REG_PWR_MGMT_1, 0x01);
    if (ret != ESP_OK) return ret;

    /* Sample-rate divider = 0 */
    ret = mpu6050_write_byte(
        dev_handle, MPU6050_REG_SMPLRT_DIV, 0x00);
    if (ret != ESP_OK) return ret;

    /* Enable DLPF: accelerometer bandwidth ~44 Hz,
       gyroscope bandwidth ~42 Hz */
    ret = mpu6050_write_byte(
        dev_handle, MPU6050_REG_CONFIG, 0x03);
    if (ret != ESP_OK) return ret;

    /* Gyroscope range: +/-250 degrees/second */
    ret = mpu6050_write_byte(
        dev_handle, MPU6050_REG_GYRO_CONFIG, 0x00);
    if (ret != ESP_OK) return ret;

    /* Accelerometer range: +/-2 g */
    ret = mpu6050_write_byte(
        dev_handle, MPU6050_REG_ACCEL_CONFIG, 0x00);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return ESP_OK;
}

/* Read raw accelerometer, temperature and gyroscope data */
esp_err_t mpu6050_read_raw(
    i2c_master_dev_handle_t dev_handle,
    mpu6050_raw_data_t *raw_data)
{
    if (raw_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer[14];

    esp_err_t ret = mpu6050_read_bytes(
        dev_handle,
        MPU6050_REG_ACCEL_XOUT_H,
        buffer,
        sizeof(buffer)
    );

    if (ret != ESP_OK) {
        return ret;
    }

    /* MPU6050 data is big-endian: high byte first */

    raw_data->accel_x =
        (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);

    raw_data->accel_y =
        (int16_t)(((uint16_t)buffer[2] << 8) | buffer[3]);

    raw_data->accel_z =
        (int16_t)(((uint16_t)buffer[4] << 8) | buffer[5]);

    /* Temperature occupies bytes 6 and 7 */
    raw_data->temp =
        (int16_t)(((uint16_t)buffer[6] << 8) | buffer[7]);

    /* Gyroscope data starts at byte 8 */
    raw_data->gyro_x =
        (int16_t)(((uint16_t)buffer[8] << 8) | buffer[9]);

    raw_data->gyro_y =
        (int16_t)(((uint16_t)buffer[10] << 8) | buffer[11]);

    raw_data->gyro_z =
        (int16_t)(((uint16_t)buffer[12] << 8) | buffer[13]);

    return ESP_OK;
}

/* Convert raw sensor values into physical units */
void mpu6050_convert_units(
    const mpu6050_raw_data_t *raw,
    mpu6050_scaled_data_t *scaled)
{
    if (raw == NULL || scaled == NULL) {
        return;
    }

    /* Accelerometer: +/-2 g, sensitivity = 16384 LSB/g */
    scaled->accel_x_g = (float)raw->accel_x / 16384.0f;
    scaled->accel_y_g = (float)raw->accel_y / 16384.0f;
    scaled->accel_z_g = (float)raw->accel_z / 16384.0f;

    /* Gyroscope: +/-250 dps, sensitivity = 131 LSB/(degree/s) */
    scaled->gyro_x_dps = (float)raw->gyro_x / 131.0f;
    scaled->gyro_y_dps = (float)raw->gyro_y / 131.0f;
    scaled->gyro_z_dps = (float)raw->gyro_z / 131.0f;

    /* Temperature conversion from the datasheet */
    scaled->temp_c = ((float)raw->temp / 340.0f) + 36.53f;
}
