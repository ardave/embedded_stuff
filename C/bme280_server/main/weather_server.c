#include <stdio.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "bme280_sensor.h"
#include "data_poster.h"
#include "led_status.h"
#include "wifi_manager.h"

static const char *TAG = "weather_server";

static void on_wifi_state_change(bool connected) {
    if (connected) {
        data_poster_start();
    } else {
        data_poster_stop();
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Weather Server starting...");

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

    // Initialize data poster
    ESP_ERROR_CHECK(data_poster_init(bme280_sensor_get_handle()));

    // Initialize WiFi and connect
    ESP_ERROR_CHECK(wifi_manager_init(on_wifi_state_change));

    // Wait for WiFi connection
    ESP_ERROR_CHECK(wifi_manager_wait_connected());

    ESP_LOGI(TAG, "WiFi connected! Posting sensor data every 15 minutes.");
}
