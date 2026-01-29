#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

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

int nmea_parse_gga(const char *sentence, gps_data_t *data);

#endif
