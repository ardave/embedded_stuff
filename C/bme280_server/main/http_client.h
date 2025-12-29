#pragma once

#include "esp_err.h"
#include "sensor_reading.pb.h"

esp_err_t http_client_post_reading(SensorReading *reading);

