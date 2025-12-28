#include <stdio.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "i2c_bus.h"
#include "bme280.h"
#include "http_server.h"
#include "led_status.h"
#include "wifi_manager.h"

static const char *TAG = "bme280_server";

// I2C configuration for BME280 (Adafruit QT Py ESP32-C3 STEMMA QT connector)
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_SDA_IO   5
#define I2C_MASTER_SCL_IO   6
#define I2C_MASTER_FREQ_HZ  100000

static bme280_handle_t bme280_sensor = NULL;

static esp_err_t bme280_sensor_init(void) {
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

static void on_wifi_state_change(bool connected) {
    if (connected) {
        http_server_init(bme280_sensor);
    } else {
        http_server_stop();
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "BME280 Server starting...");

    // Initialize NVS (required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize LED status indicator
    ESP_ERROR_CHECK(led_status_init());

    // Initialize BME280 sensor
    ESP_ERROR_CHECK(bme280_sensor_init());

    // Initialize WiFi and connect
    ESP_ERROR_CHECK(wifi_manager_init(on_wifi_state_change));

    // Wait for WiFi connection
    ESP_ERROR_CHECK(wifi_manager_wait_connected());

    ESP_LOGI(TAG, "WiFi connected! HTTP server serving BME280 sensor data.");
}
