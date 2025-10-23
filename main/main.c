/*
 * Simplified code for ESP32
 * Functions: bypass mode switching, sensor disabling, magnetometer configuration
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"


static const char *TAG = "magnetometer";

#define I2C_MASTER_SCL_IO           10                /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           11                /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0         /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          400000            /*!< I2C master clock frequency */
#define I2C_MASTER_TIMEOUT_MS       1000    

#define MPU9250_SENSOR_ADDR         0x68        /*!< Address of the MPU9250 sensor */
#define MPU9250_PWR_MGMT_1_REG_ADDR 0x6B        /*!< Register addresses of the power management register */
#define MPU9250_RESET_BIT           7

#define AK8963_ST1_REG_ADDR         0x02        /*!< Register address of the magnetometer status register */
#define AK8963_ST2_REG_ADDR         0x09        /*!< Register address of the magnetometer status register */
#define PWR_MGMT_2                  0x6C        /*!< Power Management 2 register */
#define BYPASS_EN                   0x37        /*!< Bypass enable bit in INT_PIN_CFG register */
#define CONTROL_REG1                0x0A        /*!< Control register 1 in magnetometer */
#define AK8963_HXL_REG_ADDR         0x03        /*!< Register address of the magnetometer X-axis low byte */
#define AK8963_I2C_ADDR             0x0C        /*!< I2C address of the magnetometer */
#define AK8963_ASA_REG_ADDR         0x10        /*!< Register address of the magnetometer sensitivity adjustment */

/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
static esp_err_t mpu9250_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
static esp_err_t mpu9250_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}


static esp_err_t ak8963_register_read(i2c_master_dev_handle_t mag_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(mag_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t ak8963_register_write_byte(i2c_master_dev_handle_t mag_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(mag_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief i2c master initialization
 */
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU9250_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

/**
 * Switch to bypass mode for direct access to magnetometer
 */
void set_bypass_mode(i2c_master_dev_handle_t dev_handle) {
    // Set BYPASS_EN bit in INT_PIN_CFG register (0x37)
    uint8_t current_value;
    mpu9250_register_read(dev_handle, BYPASS_EN, &current_value, 1);
    mpu9250_register_write_byte(dev_handle, BYPASS_EN, current_value | 0x02);
    ESP_LOGI(TAG, "Bypass mode enabled");
}

/**
 * Disable all sensors (accelerometer, gyroscope)
 */
void disable_sensors(i2c_master_dev_handle_t dev_handle) {
    // Set all STBY bits in PWR_MGMT_2 register (0x6C)
    mpu9250_register_write_byte(dev_handle, PWR_MGMT_2, 0x3F);
    ESP_LOGI(TAG, "All sensors disabled");
}

/**
 * Setup magnetometer in continuous reading mode at 100Hz
 */
void setup_magnetometer_continuous_mode(i2c_master_dev_handle_t mag_handle) {
    // Set continuous measurement mode at 100Hz in AK8963_CNTL1 register (0x0A)
    // Bits 4:3 = 10 (100 Hz), Bits 3:0 = 1000 (16-bit output)
    ak8963_register_write_byte(mag_handle, CONTROL_REG1, 0x16);
    ESP_LOGI(TAG, "Magnetometer set to continuous mode at 100Hz");
}

void setup_magnetometer_single_mode(i2c_master_dev_handle_t mag_handle) {
    ak8963_register_write_byte(mag_handle, CONTROL_REG1, 0x11); // Single measurement mode
}

void reset_magnetometer(i2c_master_dev_handle_t mag_handle) {
    ak8963_register_write_byte(mag_handle, CONTROL_REG1, 0x01); // Power down
    vTaskDelay(pdMS_TO_TICKS(10));
    ak8963_register_write_byte(mag_handle, CONTROL_REG1, 0x08); // Self-test mode
    vTaskDelay(pdMS_TO_TICKS(10));
    ak8963_register_write_byte(mag_handle, CONTROL_REG1, 0x00); // Power down again
}

void calibrate_magnetometer(i2c_master_dev_handle_t mag_handle) {
    // Запускаем калибровку
    ak8963_register_write_byte(mag_handle, CONTROL_REG1, 0x1F); // Fuse ROM access mode
    vTaskDelay(pdMS_TO_TICKS(10));
    
    uint8_t asa[3];
    ak8963_register_read(mag_handle, AK8963_ASA_REG_ADDR, asa, 3); // Читаем данные калибровки
    
    // Возвращаемся в рабочий режим
    ak8963_register_write_byte(mag_handle, CONTROL_REG1, 0x12);
    
    ESP_LOGI(TAG, "Magnetometer calibration data: X=%d, Y=%d, Z=%d", asa[0], asa[1], asa[2]);
}

/**
 * Read magnetometer data
 */
void read_magnetometer_data(i2c_master_dev_handle_t mag_handle) {
    uint8_t status;
    // Check data ready status (ST1 register)
    ak8963_register_read(mag_handle, AK8963_ST1_REG_ADDR, &status, 1);
    
    if (status & 0x01) {  // DRDY bit is set
        uint8_t mag_data[7];  // 6 bytes of data + 1 byte ST2
        
        // Read magnetometer data (HXL to HZH) and ST2 register
        ak8963_register_read(mag_handle, AK8963_HXL_REG_ADDR, mag_data, 7);
        
        // Check for overflow errors (HOFL bit in ST2)
        if (mag_data[6] & 0x08) {
            ESP_LOGW(TAG, "Magnetometer overflow detected");
        } else {
            // Combine high and low bytes
            int16_t mag_x = (int16_t)(mag_data[1] << 8 | mag_data[0]);
            int16_t mag_y = (int16_t)(mag_data[3] << 8 | mag_data[2]);
            int16_t mag_z = (int16_t)(mag_data[5] << 8 | mag_data[4]);
            
            ESP_LOGI(TAG, "Magnetometer: X=%d, Y=%d, Z=%d", mag_x, mag_y, mag_z);
        }
    }
}

void app_main(void) {

    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_dev_handle_t mag_handle;
    
    // Инициализация I2C
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");
    
    // Сброс MPU9250
    ESP_ERROR_CHECK(mpu9250_register_write_byte(dev_handle, MPU9250_PWR_MGMT_1_REG_ADDR, 1 << MPU9250_RESET_BIT));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Пробуждение MPU9250
    ESP_ERROR_CHECK(mpu9250_register_write_byte(dev_handle, MPU9250_PWR_MGMT_1_REG_ADDR, 0x00));
    
    // Отключаем все датчики, кроме магнитометра
    disable_sensors(dev_handle);
    
    // Переходим в режим bypass для прямого доступа к магнитометру
    set_bypass_mode(dev_handle);
    
    // Создаем устройство для магнитометра в режиме bypass
    i2c_device_config_t mag_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AK8963_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &mag_config, &mag_handle));
    
    // Сброс и калибровка магнитометра
    reset_magnetometer(mag_handle);
    calibrate_magnetometer(mag_handle);

    // Настраиваем магнитометр для непрерывного считывания
    setup_magnetometer_continuous_mode(mag_handle);
    
    ESP_LOGI(TAG, "Setup complete. Reading magnetometer data...");
    
    // Основной цикл чтения данных с магнитометра
    while (1) {
        read_magnetometer_data(mag_handle);
        vTaskDelay(pdMS_TO_TICKS(10)); // Чтение с частотой 100 Гц (10 мс)
    }
}
