#ifndef PB_SENSOR_READING_PB_H_INCLUDED
#define PB_SENSOR_READING_PB_H_INCLUDED

#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

typedef struct _SensorReading {
    float temperature_f;
    float pressure_inhg;
    float humidity_percent;
    char device_id[64];
} SensorReading;

#define SensorReading_init_default {0, 0, 0, ""}
#define SensorReading_init_zero {0, 0, 0, ""}

#define SensorReading_temperature_f_tag 1
#define SensorReading_pressure_inhg_tag 2
#define SensorReading_humidity_percent_tag 3
#define SensorReading_device_id_tag 4

#define SensorReading_FIELDLIST(X, a) \
    X(a, STATIC, SINGULAR, FLOAT, temperature_f, 1) \
    X(a, STATIC, SINGULAR, FLOAT, pressure_inhg, 2) \
    X(a, STATIC, SINGULAR, FLOAT, humidity_percent, 3) \
    X(a, STATIC, SINGULAR, STRING, device_id, 4)

#define SensorReading_CALLBACK NULL
#define SensorReading_DEFAULT NULL

extern const pb_msgdesc_t SensorReading_msg;

#define SensorReading_fields &SensorReading_msg

#define SensorReading_size 80

#endif
