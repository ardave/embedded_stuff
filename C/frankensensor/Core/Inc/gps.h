#ifndef GPS_H
#define GPS_H

#include <stdint.h>

typedef struct {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;

    uint8_t fix_quality; /* 0=none, 1=GPS, 2=DGPS */
    uint8_t satellites;

    float latitude;  /* Degrees, negative = South */
    float longitude; /* Degrees, negative = West */
    float altitude;  /* Meters above sea level */
    float hdop;

} gps_data_t;

int gps_parse_gga(const char *sentence, gps_data_t *data);
void gps_i2c_bus_recovery(void);

#endif