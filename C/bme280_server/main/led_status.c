#include "led_status.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "led_status";

// Adafruit QT Py ESP32-C3 NeoPixel pins
#define NEOPIXEL_DATA_GPIO  2
#define NEOPIXEL_POWER_GPIO 8

// RMT resolution: 10MHz = 100ns per tick
#define RMT_RESOLUTION_HZ 10000000

static rmt_channel_handle_t led_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;

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
    case 0:
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            ws2812_encoder->state = 1;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        // fall-through
    case 1:
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
    esp_err_t ret;
    ws2812_encoder_t *ws2812_encoder = calloc(1, sizeof(ws2812_encoder_t));
    if (!ws2812_encoder) {
        return ESP_ERR_NO_MEM;
    }

    ws2812_encoder->base.encode = ws2812_encode;
    ws2812_encoder->base.reset = ws2812_encoder_reset;
    ws2812_encoder->base.del = ws2812_encoder_del;

    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .level0 = 1,
            .duration0 = 4,
            .level1 = 0,
            .duration1 = 8,
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = 7,
            .level1 = 0,
            .duration1 = 6,
        },
        .flags.msb_first = 1,
    };
    ret = rmt_new_bytes_encoder(&bytes_encoder_config, &ws2812_encoder->bytes_encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create bytes encoder: %s", esp_err_to_name(ret));
        goto err_free_encoder;
    }

    rmt_copy_encoder_config_t copy_encoder_config = {};
    ret = rmt_new_copy_encoder(&copy_encoder_config, &ws2812_encoder->copy_encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create copy encoder: %s", esp_err_to_name(ret));
        goto err_del_bytes_encoder;
    }

    ws2812_encoder->reset_code = (rmt_symbol_word_t) {
        .level0 = 0,
        .duration0 = 2800,
        .level1 = 0,
        .duration1 = 2800,
    };

    *ret_encoder = &ws2812_encoder->base;
    return ESP_OK;

err_del_bytes_encoder:
    rmt_del_encoder(ws2812_encoder->bytes_encoder);
err_free_encoder:
    free(ws2812_encoder);
    return ret;
}

void led_set_color(uint8_t r, uint8_t g, uint8_t b)
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

esp_err_t led_status_init(void)
{
    esp_err_t ret;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << NEOPIXEL_POWER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set GPIO config: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = gpio_set_level(NEOPIXEL_POWER_GPIO, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set GPIO level: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = NEOPIXEL_DATA_GPIO,
        .mem_block_symbols = 64,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .trans_queue_depth = 4,
    };
    ret = rmt_new_tx_channel(&tx_chan_config, &led_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT TX channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = create_ws2812_encoder(&led_encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create WS2812 encoder: %s", esp_err_to_name(ret));
        goto err_del_channel;
    }

    ret = rmt_enable(led_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT channel: %s", esp_err_to_name(ret));
        goto err_del_encoder;
    }

    led_set_color(0, 0, 0);

    ESP_LOGI(TAG, "LED initialized");
    return ESP_OK;

err_del_encoder:
    rmt_del_encoder(led_encoder);
    led_encoder = NULL;
err_del_channel:
    rmt_del_channel(led_chan);
    led_chan = NULL;
    return ret;
}

void led_status_show_post_result(bool success)
{
    uint8_t r = success ? 0 : 255;
    uint8_t g = success ? 255 : 0;

    ESP_LOGI(TAG, "POST %s", success ? "SUCCESS" : "FAILED");

    for (int i = 0; i < 3; i++) {
        led_set_color(r, g, 0);
        vTaskDelay(pdMS_TO_TICKS(80));
        led_set_color(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(80));
    }
}
