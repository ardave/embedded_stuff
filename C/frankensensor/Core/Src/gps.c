#include "gps.h"
#include "i2c.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdlib.h>

void gps_i2c_bus_recovery(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Temporarily configure SCL (PB6) as GPIO output */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Toggle SCL 9 times to free any stuck slave */
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    /* Reinitialize I2C peripheral (reconfigures the pins) */
    HAL_I2C_DeInit(&hi2c1);
    HAL_I2C_Init(&hi2c1);
}

static const char *find_field(const char *sentence, int field_num) {
    int commas = 0;
    for (int i =0; sentence[i] != '\0'; i++) {
        if (sentence[i] == ',') {
            commas ++;
            if (commas == field_num) {
                return &sentence[i + 1];
            }
        }
    }
    return NULL;
}

int gps_parse_gga(const char *sentence, gps_data_t *data) {
    if (strncmp(sentence, "$GNGGA", 6) != 0) {
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

    return 0;
}