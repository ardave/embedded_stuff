#pragma once

#include "esp_err.h"
#include <stdbool.h>

typedef void (*wifi_state_callback_t)(bool connected);

esp_err_t wifi_manager_init(wifi_state_callback_t callback);
esp_err_t wifi_manager_wait_connected(void);
void wifi_manager_set_callback(wifi_state_callback_t callback);
