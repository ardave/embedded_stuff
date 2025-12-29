#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "bme280.h"

esp_err_t data_poster_init(bme280_handle_t sensor);
bool post_sensor_reading(void);

