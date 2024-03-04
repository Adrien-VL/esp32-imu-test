#ifndef mpu6050_h
#define mpu6050_h

#include "esp_log.h"
#include "util.h"
#include "i2c.h"

#define MPU6050_SENSOR_ADDR  0x68  /*!< slave address for MPU6050 sensor */
#define MPU6050_INT_ENABLE   0x38
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_OUT_H   0x41
#define MPU6050_TEMP_OUT_L   0x42
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_GYRO_XOUT_L  0x44
#define MPU6050_GYRO_YOUT_H  0x45
#define MPU6050_GYRO_YOUT_L  0x46
#define MPU6050_GYRO_ZOUT_H  0x47
#define MPU6050_GYRO_ZOUT_L  0x48
#define MPU6050_PWR_MGMT_1   0x6B

#define MPU6050_LSBC         340.0
#define MPU6050_TEMP_OFFSET  36.53
#define MPU6050_LSBG         16384.0
#define MPU6050_LSBDS        131.0

#define MPU6050_SAMPLE_RATE             100
#define MPU6050_SAMPLE_PERIOD           0.01f
#define MPU6050_CALIBRATION_DURATION_MS 5000

#define MPU6050_SMPLRT_DIV     0x19
#define MPU6050_CONFIG         0x1A
#define MPU6050_GYRO_CONFIG    0x1B
#define MPU6050_ACCEL_CONFIG   0x1C

/**
 * @brief Initialize MPU6050
 */
void mpu6050_init(void) {
    ESP_LOGI("MPU6050", "Initializing MPU6050");
    i2c_write_byte(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, MPU6050_PWR_MGMT_1, 0); // Wake up the MPU6050
    i2c_write_byte(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, MPU6050_SMPLRT_DIV, 9); // Sample Rate Divider for 100Hz
    i2c_write_byte(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, MPU6050_CONFIG, 3);     // DLPF_CFG for ~44Hz Gyroscope Bandwidth
    i2c_write_byte(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, MPU6050_INT_ENABLE, 1); // Enable interrupt
}

/**
 * @brief Get temperature from MPU6050
 */
float mpu6050_get_temp(void) {
    uint8_t temp_h, temp_l;
    i2c_read(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, MPU6050_TEMP_OUT_H, &temp_h, 1);
    i2c_read(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, MPU6050_TEMP_OUT_L, &temp_l, 1);
    int16_t temp = util_combine_register_values(temp_h, temp_l);
    return (temp / 340.0) + 36.53;
}

/**
 * @brief Get accel batch from MPU6050
 */
void mpu6050_get_accel(float *ax, float *ay, float *az) {
    uint8_t data[6];
    i2c_read(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, MPU6050_ACCEL_XOUT_H, data, 6);
    *ax = util_combine_register_values(data[0], data[1]) / MPU6050_LSBG;
    *ay = util_combine_register_values(data[2], data[3]) / MPU6050_LSBG;
    *az = util_combine_register_values(data[4], data[5]) / MPU6050_LSBG;
}

/**
 * @brief Get gyro batch from MPU6050
 */
void mpu6050_get_gyro(float *gx, float *gy, float *gz) {
    uint8_t data[6];
    i2c_read(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, MPU6050_GYRO_XOUT_H, data, 6);
    *gx = util_combine_register_values(data[0], data[1]) / MPU6050_LSBDS;
    *gy = util_combine_register_values(data[2], data[3]) / MPU6050_LSBDS;
    *gz = util_combine_register_values(data[4], data[5]) / MPU6050_LSBDS;
}

#endif