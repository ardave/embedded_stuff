#pragma once

#include "esp_err.h"
#include "bme280.h"

esp_err_t http_server_init(bme280_handle_t sensor);
void http_server_stop(void);