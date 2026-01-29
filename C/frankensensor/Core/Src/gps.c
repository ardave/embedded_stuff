#include "gps.h"
#include "i2c.h"
#include "stm32f4xx_hal.h"

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
