#ifndef i2c_h
#define i2c_h

#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO    22  /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO    21  /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM       I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ   100000     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0   /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0   /*!< I2C master doesn't need buffer */
#define WRITE_BIT            I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT             I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN         0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS        0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL              0x0     /*!< I2C ack value */
#define NACK_VAL             0x1     /*!< I2C nack value */

/**
 * @brief i2c master initialization
 */
esp_err_t i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief Generic function to write a byte to an I2C device
 */
esp_err_t i2c_write_byte(i2c_port_t i2c_num, uint8_t device_address, uint8_t register_address, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_address << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, register_address, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Generic function to read multiple bytes from an I2C device
 */
esp_err_t i2c_read(i2c_port_t i2c_num, uint8_t device_address, uint8_t register_address, uint8_t *data, size_t length) {
    if (length == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_address << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, register_address, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_address << 1) | READ_BIT, ACK_CHECK_EN);
    if (length > 1) {
        i2c_master_read(cmd, data, length - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data + length - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

#endif