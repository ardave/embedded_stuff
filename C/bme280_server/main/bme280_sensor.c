#include "bme280_sensor.h"
#include "i2c_bus.h"
#include "esp_log.h"

static const char *TAG = "bme280_sensor";

// I2C configuration for BME280 (Adafruit QT Py ESP32-C3 STEMMA QT connector)
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_SDA_IO   5
#define I2C_MASTER_SCL_IO   6
#define I2C_MASTER_FREQ_HZ  100000

static bme280_handle_t bme280_sensor = NULL;

esp_err_t bme280_sensor_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = true,
        .scl_pullup_en = true,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_bus_handle_t i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &conf);
    if (i2c_bus == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C bus");
        return ESP_FAIL;
    }

    // Adafruit BME280 boards typically use address 0x77
    bme280_sensor = bme280_create(i2c_bus, 0x77);
    if (bme280_sensor == NULL) {
        ESP_LOGE(TAG, "Failed to create BME280 handle");
        return ESP_FAIL;
    }

    esp_err_t ret = bme280_default_init(bme280_sensor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BME280: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "BME280 sensor initialized successfully");
    return ESP_OK;
}

bme280_handle_t bme280_sensor_get_handle(void) {
    return bme280_sensor;
}
