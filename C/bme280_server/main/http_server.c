#include <stdio.h>
#include "esp_http_server.h"
#include "esp_log.h"
#include "http_server.h"

static const char *TAG = "http_server";
static httpd_handle_t server = NULL;
static bme280_handle_t bme280 = NULL;

static esp_err_t sensor_handler(httpd_req_t *req) {
    float temperature = 0, pressure = 0, humidity = 0;
    char response[128];

    esp_err_t ret = bme280_read_temperature(bme280, &temperature);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read temperature: %s", esp_err_to_name(ret));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ret = bme280_read_pressure(bme280, &pressure);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read pressure: %s", esp_err_to_name(ret));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ret = bme280_read_humidity(bme280, &humidity);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read humidity: %s", esp_err_to_name(ret));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    float temperature_f = temperature * 9.0f / 5.0f + 32.0f;
    float pressure_inhg = pressure * 0.02953f;

    snprintf(response, sizeof(response),
             "Temperature: %.1f F\nPressure: %.2f inHg\nHumidity: %.1f %%\n",
             temperature_f, pressure_inhg, humidity);

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t http_server_init(bme280_handle_t sensor) {
    if (server != NULL) {
        ESP_LOGW(TAG, "HTTP server already running");
        return ESP_ERR_INVALID_STATE;
    }

    bme280 = sensor;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP Server");
        return ESP_FAIL;
    }

    httpd_uri_t uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = sensor_handler
    };

    if (httpd_register_uri_handler(server, &uri) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register uri '/'.");
        httpd_stop(server);
        server = NULL;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Successfully initialized HTTP Server");

    return ESP_OK;
}

void http_server_stop(void) {
    if (server != NULL) {
        ESP_LOGI(TAG, "Stopping HTTP Server");
        httpd_stop(server);
        server = NULL;
    }
}

