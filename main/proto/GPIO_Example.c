#include <stdio.h>
#include "freertos/FreeRTOS.h"  // Includes FreeRTOS essentials
#include "freertos/task.h"      // Specific task-related functions
#include "i2c.h"
#include "mpu6050.h"
#include "esp_log.h"
#include "Fusion.h"
#include "math.h"

static const char *TAG = "MPU6050";

/**
 * @brief Main function to get MPU6050 data and print it
 */
void app_main(void) {
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    mpu6050_init();

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    // Calibration
    FusionVector gyroOffset = {.axis={0.0f, 0.0f, 0.0f}};
    FusionVector accelOffset = {.axis={0.0f, 0.0f, 0.0f}};
    int sampleCount = 0;

    TickType_t calibrationEndTime = xTaskGetTickCount() + pdMS_TO_TICKS(CALIBRATION_DURATION_MS);
    while (xTaskGetTickCount() < calibrationEndTime) {
        float ax, ay, az, gx, gy, gz;
        mpu6050_get_accel(&ax, &ay, &az);
        mpu6050_get_gyro(&gx, &gy, &gz);

        gyroOffset.axis.x += gx;
        gyroOffset.axis.y += gy;
        gyroOffset.axis.z += gz;
        accelOffset.axis.x += ax;
        accelOffset.axis.y += ay;
        accelOffset.axis.z += az;
        sampleCount++;

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD * 1000));
    }

    gyroOffset.axis.x /= sampleCount;
    gyroOffset.axis.y /= sampleCount;
    gyroOffset.axis.z /= sampleCount;
    accelOffset.axis.x /= sampleCount;
    accelOffset.axis.y /= sampleCount;
    accelOffset.axis.z /= sampleCount;

    ESP_LOGI("MPU6050", " ogx %0.1f, ogy %0.1f, ogz %0.1f,\n oax %0.1f, oay %0.1f, oaz %0.1f",
        gyroOffset.axis.x, 
        gyroOffset.axis.y, 
        gyroOffset.axis.z,
        accelOffset.axis.x,
        accelOffset.axis.y,
        accelOffset.axis.z);

    // Main loop
    while (1) {
        float ax, ay, az, gx, gy, gz;
        mpu6050_get_accel(&ax, &ay, &az);
        mpu6050_get_gyro(&gx, &gy, &gz);

        // Apply offsets
        gx -= gyroOffset.axis.x;
        gy -= gyroOffset.axis.y;
        gz -= gyroOffset.axis.z;
        ax -= accelOffset.axis.x;
        ay -= accelOffset.axis.y;
        az -= accelOffset.axis.z - 1; // Correct for gravity assuming the device is stationary and facing up

        FusionVector gyroscope = {.axis={gx, gy, gz}};
        FusionVector accelerometer = {.axis={ax, ay, az}};

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

        ESP_LOGI(TAG, "Roll %0.1f, Pitch %0.1f, Yaw %0.1f, X %0.1f, Y %0.1f, Z %0.1f",
                euler.angle.roll, 
                euler.angle.pitch, 
                euler.angle.yaw,
                earth.axis.x,
                earth.axis.y,
                earth.axis.z);

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD * 1000));
    }
}

// void app_main(void) {
//     ESP_ERROR_CHECK(i2c_master_init());
//     ESP_LOGI(TAG, "I2C initialized successfully");
// 
//     mpu6050_init();
// 
//     while (1) {
//         float temperature = mpu6050_get_temp();
//         ESP_LOGI(TAG, "Temperature: %.2f °C", temperature);
// 
//         float ax, ay, az;
//         mpu6050_get_accel(&ax, &ay, &az);
//         ESP_LOGI(TAG, "Accel: X=%.2f g, Y=%.2f g, Z=%.2f g", ax, ay, az);
// 
//         float gx, gy, gz;
//         mpu6050_get_gyro(&gx, &gy, &gz);
//         ESP_LOGI(TAG, "Gyro: X=%.2f °/s, Y=%.2f °/s, Z=%.2f °/s", gx, gy, gz);
// 
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }
// }