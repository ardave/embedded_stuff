#include "esp_http_server.h"
#include "esp_log.h"
#include "http_server.h"

static const char *TAG = "http_server";
static httpd_handle_t server = NULL;

static esp_err_t hello_handler(httpd_req_t *req) {
    const char *response = "hello, world!";
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t http_server_init(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP Server");
        return ESP_FAIL;
    }

    httpd_uri_t uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = hello_handler
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

