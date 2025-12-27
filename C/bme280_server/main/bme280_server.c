#include <stdio.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "http_server.h"
#include "led_status.h"
#include "wifi_manager.h"

static const char *TAG = "bme280_server";

static void on_wifi_state_change(bool connected) {
    if (connected) {
        http_server_init();
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

    // Initialize WiFi and connect
    ESP_ERROR_CHECK(wifi_manager_init(on_wifi_state_change));

    // Wait for WiFi connection
    ESP_ERROR_CHECK(wifi_manager_wait_connected());

    ESP_LOGI(TAG, "WiFi connected! Ready for BME280 sensor operations.");

    // TODO: Add BME280 sensor reading and HTTP server here
}
