#pragma once

#include "esp_err.h"
#include "bme280.h"

/**
 * Initialize the BME280 sensor on the I2C bus.
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bme280_sensor_init(void);

/**
 * Get the BME280 sensor handle for reading data.
 *
 * @return The sensor handle, or NULL if not initialized
 */
bme280_handle_t bme280_sensor_get_handle(void);

