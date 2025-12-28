#ifndef HTTP_CLIENT_H
#define HTTP_CLIENT_H

#include "esp_err.h"
#include "sensor_reading.pb.h"

esp_err_t http_client_post_reading(SensorReading *reading);

#endif // HTTP_CLIENT_H
