#include <string.h>
#include "esp_log.h"
#include "data_poster.h"
#include "http_client.h"
#include "sensor_reading.pb.h"

static const char *TAG = "data_poster";

static bme280_handle_t bme280 = NULL;

bool post_sensor_reading(void) {
    float temperature = 0, pressure = 0, humidity = 0;

    if (bme280 == NULL) {
        ESP_LOGE(TAG, "data_poster has not been initialized!");
        return false;
    }

    esp_err_t ret = bme280_read_temperature(bme280, &temperature);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read temperature: %s", esp_err_to_name(ret));
        return false;
    }

    ret = bme280_read_pressure(bme280, &pressure);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read pressure: %s", esp_err_to_name(ret));
        return false;
    }

    ret = bme280_read_humidity(bme280, &humidity);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read humidity: %s", esp_err_to_name(ret));
        return false;
    }

    float temperature_f = temperature * 9.0f / 5.0f + 32.0f;
    float pressure_inhg = pressure * 0.02953f;

    SensorReading reading = SensorReading_init_zero;
    reading.temperature_f = temperature_f;
    reading.pressure_inhg = pressure_inhg;
    reading.humidity_percent = humidity;
    strncpy(reading.device_id, DEVICE_ID, sizeof(reading.device_id) - 1);

    ESP_LOGI(TAG, "Posting reading: T=%.1fF, P=%.2finHg, H=%.1f%%, id=%s",
             reading.temperature_f, reading.pressure_inhg, reading.humidity_percent,
             reading.device_id);

    ret = http_client_post_reading(&reading);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to post sensor reading");
        return false;
    }

    return true;
}

esp_err_t data_poster_init(bme280_handle_t sensor) {
    if (sensor == NULL) {
        ESP_LOGE(TAG, "Invalid sensor handle");
        return ESP_ERR_INVALID_ARG;
    }
    bme280 = sensor;
    ESP_LOGI(TAG, "Data poster initialized");
    return ESP_OK;
}
