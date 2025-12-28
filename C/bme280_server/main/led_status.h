#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t led_status_init(void);
void led_set_color(uint8_t r, uint8_t g, uint8_t b);
void led_status_show_post_result(bool success);
