#include "led_status.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "led_status";

// Adafruit QT Py ESP32-C3 NeoPixel pins
#define NEOPIXEL_DATA_GPIO  2
#define NEOPIXEL_POWER_GPIO 8

// RMT resolution: 10MHz = 100ns per tick
#define RMT_RESOLUTION_HZ 10000000

static rmt_channel_handle_t led_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;
static led_state_t current_state = LED_STATE_OFF;
static TaskHandle_t led_task_handle = NULL;

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} ws2812_encoder_t;

static size_t ws2812_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                            const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{
    ws2812_encoder_t *ws2812_encoder = __containerof(encoder, ws2812_encoder_t, base);
    rmt_encoder_handle_t bytes_encoder = ws2812_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = ws2812_encoder->copy_encoder;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    switch (ws2812_encoder->state) {
    case 0: // Send RGB data
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            ws2812_encoder->state = 1;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        // fall-through
    case 1: // Send reset code
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &ws2812_encoder->reset_code,
                                                 sizeof(ws2812_encoder->reset_code), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            ws2812_encoder->state = RMT_ENCODING_RESET;
            state |= RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
    }
out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t ws2812_encoder_reset(rmt_encoder_t *encoder)
{
    ws2812_encoder_t *ws2812_encoder = __containerof(encoder, ws2812_encoder_t, base);
    rmt_encoder_reset(ws2812_encoder->bytes_encoder);
    rmt_encoder_reset(ws2812_encoder->copy_encoder);
    ws2812_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

static esp_err_t ws2812_encoder_del(rmt_encoder_t *encoder)
{
    ws2812_encoder_t *ws2812_encoder = __containerof(encoder, ws2812_encoder_t, base);
    rmt_del_encoder(ws2812_encoder->bytes_encoder);
    rmt_del_encoder(ws2812_encoder->copy_encoder);
    free(ws2812_encoder);
    return ESP_OK;
}

static esp_err_t create_ws2812_encoder(rmt_encoder_handle_t *ret_encoder)
{
    ws2812_encoder_t *ws2812_encoder = calloc(1, sizeof(ws2812_encoder_t));
    if (!ws2812_encoder) {
        return ESP_ERR_NO_MEM;
    }

    ws2812_encoder->base.encode = ws2812_encode;
    ws2812_encoder->base.reset = ws2812_encoder_reset;
    ws2812_encoder->base.del = ws2812_encoder_del;

    // Create bytes encoder for RGB data
    // At 10MHz (RMT_RESOLUTION_HZ), 1 tick = 100ns
    // T0H = 350ns = 3.5 ticks ~= 4
    // T0L = 800ns = 8 ticks
    // T1H = 700ns = 7 ticks
    // T1L = 600ns = 6 ticks
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .level0 = 1,
            .duration0 = 4,  // ~400ns high
            .level1 = 0,
            .duration1 = 8,  // ~800ns low
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = 7,  // ~700ns high
            .level1 = 0,
            .duration1 = 6,  // ~600ns low
        },
        .flags.msb_first = 1,
    };
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&bytes_encoder_config, &ws2812_encoder->bytes_encoder));

    // Create copy encoder for reset signal
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_encoder_config, &ws2812_encoder->copy_encoder));

    // Reset code (low for 280us = 2800 ticks at 10MHz)
    ws2812_encoder->reset_code = (rmt_symbol_word_t) {
        .level0 = 0,
        .duration0 = 2800,
        .level1 = 0,
        .duration1 = 2800,
    };

    *ret_encoder = &ws2812_encoder->base;
    return ESP_OK;
}

static void set_pixel(uint8_t r, uint8_t g, uint8_t b)
{
    if (!led_chan || !led_encoder) return;

    // WS2812 uses GRB order
    uint8_t pixel_data[3] = {g, r, b};

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    rmt_transmit(led_chan, led_encoder, pixel_data, sizeof(pixel_data), &tx_config);
    rmt_tx_wait_all_done(led_chan, portMAX_DELAY);
}

static void led_task(void *arg)
{
    int brightness = 0;
    int direction = 5;
    int blink_state = 0;

    while (1) {
        switch (current_state) {
            case LED_STATE_OFF:
                set_pixel(0, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(100));
                break;

            case LED_STATE_CONNECTING:
                // Yellow pulsing
                brightness += direction;
                if (brightness >= 255) {
                    brightness = 255;
                    direction = -5;
                } else if (brightness <= 20) {
                    brightness = 20;
                    direction = 5;
                }
                set_pixel(brightness, brightness * 180 / 255, 0);
                vTaskDelay(pdMS_TO_TICKS(20));
                break;

            case LED_STATE_CONNECTED:
                // Green solid
                set_pixel(0, 255, 0);
                vTaskDelay(pdMS_TO_TICKS(100));
                break;

            case LED_STATE_DISCONNECTED:
                // Red blinking
                blink_state = !blink_state;
                if (blink_state) {
                    set_pixel(255, 0, 0);
                } else {
                    set_pixel(0, 0, 0);
                }
                vTaskDelay(pdMS_TO_TICKS(250));
                break;
        }
    }
}

esp_err_t led_status_init(void)
{
    // Enable NeoPixel power (GPIO8 must be HIGH)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << NEOPIXEL_POWER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(NEOPIXEL_POWER_GPIO, 1);

    // Give power time to stabilize
    vTaskDelay(pdMS_TO_TICKS(10));

    // Configure RMT TX channel
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = NEOPIXEL_DATA_GPIO,
        .mem_block_symbols = 64,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .trans_queue_depth = 4,
    };
    esp_err_t ret = rmt_new_tx_channel(&tx_chan_config, &led_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT TX channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create WS2812 encoder
    ret = create_ws2812_encoder(&led_encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create WS2812 encoder: %s", esp_err_to_name(ret));
        return ret;
    }

    // Enable channel
    ret = rmt_enable(led_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Clear LED
    set_pixel(0, 0, 0);

    // Create LED animation task
    xTaskCreate(led_task, "led_task", 2048, NULL, 5, &led_task_handle);

    ESP_LOGI(TAG, "LED status initialized");
    return ESP_OK;
}

void led_set_state(led_state_t state)
{
    current_state = state;
    ESP_LOGI(TAG, "LED state changed to %d", state);
}
