#pragma once

#include "esp_err.h"

typedef enum {
    LED_STATE_OFF,
    LED_STATE_CONNECTING,    // Yellow pulsing
    LED_STATE_CONNECTED,     // Green solid
    LED_STATE_DISCONNECTED,  // Red blinking
} led_state_t;

esp_err_t led_status_init(void);
void led_set_state(led_state_t state);
