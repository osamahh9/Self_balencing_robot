#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "mpu6050.h"

static const char *TAG = "MPU6050_EXAMPLE";

// Define I2C settings
#define I2C_MASTER_SCL_IO   GPIO_NUM_20
#define I2C_MASTER_SDA_IO   GPIO_NUM_21
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  100000

static mpu6050_handle_t mpu6050 = NULL;

/**
 * @brief I2C master initialization
 */
static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void app_main(void)
{
    // 1. Initialize I2C communication
    i2c_master_init();
    ESP_LOGI(TAG, "I2C initialized successfully");

    // 2. Create the MPU6050 device handle
    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);

    // 3. Wake up the MPU6050
    mpu6050_wake_up(mpu6050);
    ESP_LOGI(TAG, "MPU6050 woken up");

    // 4. Main loop to read sensor data
    while (1) {
        mpu6050_acce_value_t acce;
        mpu6050_gyro_value_t gyro;

        // Read accelerometer and gyroscope data
        mpu6050_get_acce(mpu6050, &acce);
        mpu6050_get_gyro(mpu6050, &gyro);

        // Print the data
        ESP_LOGI(TAG, "Acce: X=%.2f, Y=%.2f, Z=%.2f || Gyro: X=%.2f, Y=%.2f, Z=%.2f",
                 acce.acce_x, acce.acce_y, acce.acce_z,
                 gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

        // Wait for 1 second before the next read
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}