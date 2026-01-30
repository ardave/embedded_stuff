#include "nmea_parser.h"
#include <string.h>
#include <stdlib.h>

/* Convert a hex character to its integer value (0-15), or -1 if invalid */
static int hex_char_to_int(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/* Validate NMEA checksum: XOR of all bytes between '$' and '*' */
static int validate_checksum(const char *sentence) {
    if (sentence == NULL || sentence[0] != '$') {
        return -1;
    }

    uint8_t calculated = 0;
    int i = 1;  /* Start after '$' */

    /* XOR all characters until '*' or end of string */
    while (sentence[i] != '\0' && sentence[i] != '*') {
        calculated ^= (uint8_t)sentence[i];
        i++;
    }

    /* Must have found '*' followed by two hex digits */
    if (sentence[i] != '*') {
        return -1;
    }

    int high = hex_char_to_int(sentence[i + 1]);
    int low = hex_char_to_int(sentence[i + 2]);

    if (high < 0 || low < 0) {
        return -1;
    }

    uint8_t expected = (uint8_t)((high << 4) | low);

    return (calculated == expected) ? 0 : -1;
}

static const char *find_field(const char *sentence, int field_num) {
    int commas = 0;
    for (int i = 0; sentence[i] != '\0'; i++) {
        if (sentence[i] == ',') {
            commas ++;
            if (commas == field_num) {
                return &sentence[i + 1];
            }
        }
    }
    return NULL;
}

int nmea_parse_gga(const char *sentence, gps_data_t *data) {
    if (sentence == NULL || data == NULL) {
        return -1;
    }

    if (strncmp(sentence, "$GNGGA", 6) != 0) {
        return -1;
    }

    /* Validate checksum before parsing */
    if (validate_checksum(sentence) != 0) {
        return -1;
    }

    memset(data, 0, sizeof(gps_data_t));

    /* Field 1: Time (HHMMSS.sss) **/
    const char *field = find_field(sentence, 1);

    if (field != NULL && field[0] != ',') {
        data->hours = (field[0] - '0') * 10 + (field[1] - '0');
        data->minutes = (field[2] - '0') * 10 + (field[3] - '0');
        data->seconds = (field[4] - '0') * 10 + (field[5] - '0');
    }

    /* Field 6: Fix quality (0-none, 1=GPS, 2=DGPS) */
    field = find_field(sentence, 6);
    if (field != NULL && field[0] != ',') {
        data->fix_quality = field[0] - '0';
    }

    /* Field 7: Number of satellites */
    field = find_field(sentence, 7);
    if (field != NULL && field[0] != ',') {
        data->satellites = (field[0] - '0');
        if (field[1] != ',') {
            data->satellites = data-> satellites * 10 + (field[1] - '0');
        }
    }

    /* Field 2: Latitude (DDMM.MMMM) */
    field = find_field(sentence, 2);
    if (field != NULL && field[0] != ',') {
        float degrees = (field[0] - '0') * 10 + (field[1] - '0');
        float minutes = atof(&field[2]);
        data->latitude = degrees + (minutes / 60.0f);

        /* Field 3: N/S indicator */
        const char *ns = find_field(sentence, 3);
        if (ns != NULL && ns[0] == 'S') {
            data->latitude = -data->latitude;
        }
    }

    /* Field 4: Longitude (DDDMM.MMMM) */
    field = find_field(sentence, 4);
    if (field != NULL && field[0] != ',') {
        float degrees = (field[0] - '0') * 100 + (field[1] - '0') * 10 + (field[2] - '0');
        float minutes = atof(&field[3]);
        data->longitude = degrees + (minutes / 60.0f);

        /* Field 5: E/W indicator */
        const char *ew = find_field(sentence, 5);
        if (ew != NULL && ew[0] == 'W') {
            data->longitude = -data->longitude;
        }
    }

    /* Field 8: HDOP (horizontal dilution of precision) */
    field = find_field(sentence, 8);
    if (field != NULL && field[0] != ',') {
        data->hdop = atof(field);
    }

    /* Field 9: Altitude in meters */
    field = find_field(sentence, 9);
    if (field != NULL && field[0] != ',') {
        data->altitude = atof(field);
    }

    return 0;
}
