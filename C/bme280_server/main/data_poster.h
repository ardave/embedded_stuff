#ifndef DATA_POSTER_H
#define DATA_POSTER_H

#include "esp_err.h"
#include "bme280.h"

esp_err_t data_poster_init(bme280_handle_t sensor);
void data_poster_start(void);
void data_poster_stop(void);

#endif // DATA_POSTER_H
