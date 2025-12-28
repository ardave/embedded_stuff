#include <string.h>
#include "esp_log.h"
#include "esp_http_client.h"
#include "http_client.h"
#include "pb_encode.h"
#include "sensor_reading.pb.h"

static const char *TAG = "http_client";

#define MAX_RETRIES 3
#define INITIAL_BACKOFF_MS 1000

esp_err_t http_client_post_reading(SensorReading *reading) {
    uint8_t buffer[SensorReading_size];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    if (!pb_encode(&stream, SensorReading_fields, reading)) {
        ESP_LOGE(TAG, "Failed to encode protobuf: %s", PB_GET_ERROR(&stream));
        return ESP_FAIL;
    }

    size_t message_length = stream.bytes_written;
    ESP_LOGI(TAG, "Encoded protobuf message: %zu bytes", message_length);

    // WARNING: Skipping cert verification for testing - add proper cert for production
    esp_http_client_config_t config = {
        .url = API_ENDPOINT,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 10000,
        .skip_cert_common_name_check = true,
    };

    int backoff_ms = INITIAL_BACKOFF_MS;

    for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
        esp_http_client_handle_t client = esp_http_client_init(&config);
        if (client == NULL) {
            ESP_LOGE(TAG, "Failed to initialize HTTP client");
            return ESP_FAIL;
        }

        esp_http_client_set_header(client, "Content-Type", "application/x-protobuf");
        esp_http_client_set_post_field(client, (const char *)buffer, message_length);

        esp_err_t err = esp_http_client_perform(client);

        if (err == ESP_OK) {
            int status_code = esp_http_client_get_status_code(client);
            if (status_code >= 200 && status_code < 300) {
                ESP_LOGI(TAG, "POST successful, status code: %d", status_code);
                esp_http_client_cleanup(client);
                return ESP_OK;
            }
            ESP_LOGW(TAG, "POST returned status code: %d (attempt %d/%d)",
                     status_code, attempt, MAX_RETRIES);
        } else {
            ESP_LOGW(TAG, "POST failed: %s (attempt %d/%d)",
                     esp_err_to_name(err), attempt, MAX_RETRIES);
        }

        esp_http_client_cleanup(client);

        if (attempt < MAX_RETRIES) {
            ESP_LOGI(TAG, "Retrying in %d ms...", backoff_ms);
            vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            backoff_ms *= 2;
        }
    }

    ESP_LOGE(TAG, "All %d retry attempts failed", MAX_RETRIES);
    return ESP_FAIL;
}
