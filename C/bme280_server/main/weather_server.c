#include <stdio.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "bme280_sensor.h"
#include "data_poster.h"
#include "led_status.h"
#include "wifi_manager.h"

static const char *TAG = "weather_server";

#define SLEEP_DURATION_US (15 * 60 * 1000000ULL)  // 15 minutes in microseconds

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

    // Initialize LED and show yellow while connecting
    ESP_ERROR_CHECK(led_status_init());
    led_set_color(255, 180, 0);  // Yellow

    // Initialize BME280 sensor
    ESP_ERROR_CHECK(bme280_sensor_init());

    // Connect to WiFi
    ESP_ERROR_CHECK(wifi_manager_init());
    ESP_ERROR_CHECK(wifi_manager_wait_connected());

    // Initialize data poster and sync time
    ESP_ERROR_CHECK(data_poster_init(bme280_sensor_get_handle()));
    data_poster_sync_time();

    // POST sensor reading
    bool success = post_sensor_reading();

    // Show result on LED (blocking animation)
    led_status_show_post_result(success);

    // Enter deep sleep
    ESP_LOGI(TAG, "Entering deep sleep for 15 minutes...");
    esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);
    esp_deep_sleep_start();
}
