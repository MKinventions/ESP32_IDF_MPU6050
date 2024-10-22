#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000 //100000
#define MPU6050_SENSOR_ADDR         0x68
#define MPU6050_ACCEL_XOUT_H        0x3B
#define MPU6050_TEMP_OUT_H          0x41
#define MPU6050_GYRO_XOUT_H         0x43
#define MPU6050_PWR_MGMT_1_REG      0x6B

static const char *TAG = "MPU6050";

float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float temperature_celsius, temperature_fahrenheit;

// I2C initialization
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Write a byte to MPU6050
static esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Read multiple bytes from MPU6050
static esp_err_t mpu6050_read_bytes(uint8_t start_reg_addr, uint8_t *data, size_t length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, start_reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, length, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}


// Read accelerometer, gyroscope, and temperature data
static void read_mpu6050_data(void) {
    uint8_t data[14];
    mpu6050_read_bytes(MPU6050_ACCEL_XOUT_H, data, 14);

    // Convert accelerometer data
    int16_t readRaw_accel_x = ((int16_t)data[0] << 8) | data[1];
    int16_t readRaw_accel_y = ((int16_t)data[2] << 8) | data[3];
    int16_t readRaw_accel_z = ((int16_t)data[4] << 8) | data[5];

    // Convert temperature data
    int16_t readRaw_temperature = ((int16_t)data[6] << 8) | data[7];

    // Convert gyroscope data
    int16_t readRaw_gyro_x = ((int16_t)data[8] << 8) | data[9];
    int16_t readRaw_gyro_y = ((int16_t)data[10] << 8) | data[11];
    int16_t readRaw_gyro_z = ((int16_t)data[12] << 8) | data[13];


    //processed data
    accel_x = readRaw_accel_x / 16384.0;
    accel_y = readRaw_accel_y / 16384.0;
    accel_z = readRaw_accel_z / 16384.0;
    temperature_celsius = (readRaw_temperature / 340.0) + 36.53;
    temperature_fahrenheit = (temperature_celsius * 9.0 / 5.0) + 32.0;
    gyro_x = readRaw_gyro_x / 131.0;
    gyro_y = readRaw_gyro_y / 131.0;
    gyro_z = readRaw_gyro_z / 131.0;

}


// Task to read and log MPU6050 data
void mpu6050_task(void *arg) {
    while (1) {
        read_mpu6050_data();

        // Log sensor data
        ESP_LOGI(TAG, "Accel (g): X=%.2f, Y=%.2f, Z=%.2f", accel_x, accel_y, accel_z);
        ESP_LOGI(TAG, "Gyro (DPS): X=%.2f, Y=%.2f, Z=%.2f", gyro_x, gyro_y, gyro_z);
//
//        // Log sensor data
        ESP_LOGI(TAG, "Temperature: %.2f degC", temperature_celsius);
        ESP_LOGI(TAG, "Temperature: %.2f degF", temperature_fahrenheit);


//        if((accel_x - 0.11) <= -0.50){
//        	printf("***Leaning left***\n");
//        }else if((accel_x - 0.11) >= 0.50){
//        	printf("***Leaning right***\n");
//        }
//        if(accel_y <= -0.50){
//        	printf("***Leaning back***\n");
//        }else if(accel_y >= 0.50){
//        	printf("***Leaning front***\n");
//        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}

void app_main(void) {
    i2c_master_init();
    mpu6050_write_byte(MPU6050_PWR_MGMT_1_REG, 0x00);  // Wake up MPU6050

    // Create the MPU6050 task
    xTaskCreate(mpu6050_task, "mpu6050_task", 2048, NULL, 5, NULL);
}
