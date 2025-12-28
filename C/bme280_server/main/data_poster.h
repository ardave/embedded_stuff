#ifndef DATA_POSTER_H
#define DATA_POSTER_H

#include <stdbool.h>
#include "esp_err.h"
#include "bme280.h"

esp_err_t data_poster_init(bme280_handle_t sensor);
bool post_sensor_reading(void);

#endif // DATA_POSTER_H
