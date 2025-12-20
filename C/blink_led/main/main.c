#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define BLINK_GPIO 8
#define SWITCH_GPIO 4  // A0 pin on QT Py C3

void app_main(void)
{
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    gpio_reset_pin(SWITCH_GPIO);
    gpio_set_direction(SWITCH_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SWITCH_GPIO, GPIO_PULLUP_ONLY);

    while (1) {
        int switch_pressed = (gpio_get_level(SWITCH_GPIO) == 0);
        gpio_set_level(BLINK_GPIO, switch_pressed);
        vTaskDelay(10 / portTICK_PERIOD_MS);  // debounce delay
    }
}
