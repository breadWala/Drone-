
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#define MPU6050_I2C_ADDR          0x68

#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_WHO_AM_I      0x75

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
} mpu6050_raw_data_t;

typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    float temp_c;
} mpu6050_scaled_data_t;

esp_err_t mpu6050_init(i2c_master_dev_handle_t dev_handle);

esp_err_t mpu6050_read_raw(
    i2c_master_dev_handle_t dev_handle,
    mpu6050_raw_data_t *raw_data
);

void mpu6050_convert_units(
    const mpu6050_raw_data_t *raw,
    mpu6050_scaled_data_t *scaled
);

