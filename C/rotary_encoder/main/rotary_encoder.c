#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "soc/gpio_reg.h"

/*
  Wiring:
  ┌─────────────┬───────────┬───────────────┐
  │ Encoder Pin │ QT Py Pin │ ESP32-C3 GPIO │
  ├─────────────┼───────────┼───────────────┤
  │ +           │ 3V        │ -             │
  ├─────────────┼───────────┼───────────────┤
  │ GND         │ GND       │ -             │
  ├─────────────┼───────────┼───────────────┤
  │ CLK         │ A0        │ GPIO4         │
  ├─────────────┼───────────┼───────────────┤
  │ DT          │ A1        │ GPIO3         │
  ├─────────────┼───────────┼───────────────┤
  │ SW          │ A2        │ GPIO1         │
  └─────────────┴───────────┴───────────────┘
*/

#define CLK_PIN GPIO_NUM_4 // A0 on QT Py
#define DT_PIN  GPIO_NUM_3 // A1 on QT Py
#define SW_PIN  GPIO_NUM_1 // A2 on QT Py
#define BUTTON_DEBOUNCE_TIME_US 250000  // 250ms debounce
#define ENCODER_QUEUE_SIZE 64

void encoder_task(void *pvParameters);
void button_task(void *pvParameters);
void configure_gpio(void);
const char* encoder_transition_type(uint8_t last_state, uint8_t new_state);

QueueHandle_t encoder_queue = NULL;
TaskHandle_t encoder_task_handle = NULL;
TaskHandle_t button_task_handle = NULL;

void app_main(void)
{
    configure_gpio();

    encoder_queue = xQueueCreate(ENCODER_QUEUE_SIZE, sizeof(uint8_t));

    xTaskCreate(
        encoder_task,
        "encoder_task",
        2048,
        NULL,
        5,
        &encoder_task_handle
    );

    xTaskCreate(
        button_task,
        "button_task",
        2048,
        NULL,
        5,
        &button_task_handle
    );
}

void IRAM_ATTR encoderISR(void *arg) {
    static uint8_t last_queued_state = 0xFF;  // Initialize to invalid state

    uint32_t gpio_in = REG_READ(GPIO_IN_REG);
    uint8_t state = (((gpio_in >> CLK_PIN) & 1) << 1) | ((gpio_in >> DT_PIN) & 1);

    if (state != last_queued_state) {
        last_queued_state = state;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(encoder_queue, &state, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void IRAM_ATTR buttonISR(void *arg) {
    static int64_t last_edge_time = 0;
    int64_t now = esp_timer_get_time();

    if ((now - last_edge_time) > BUTTON_DEBOUNCE_TIME_US) {
        last_edge_time = now;

        // Only trigger on released (1) -> pressed (0) transition
        // Why do we need to notify that a higher priority task was NOT awoken?
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(button_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

const char* encoder_transition_type(uint8_t last_state, uint8_t new_state) {
    // Legal gray code transitions change exactly one bit
    uint8_t diff = last_state ^ new_state;
    return (diff == 1 || diff == 2) ? "legal" : "illegal";
}

void encoder_task(void *pvParameters) {
    uint8_t new_state;
    uint8_t last_state = 0;
    int32_t position = 0;

    // State table for gray code: [last_state << 2 | new_state]
    static const int8_t state_table[16] = {
         0, -1,  1,  0,   // from 00
         1,  0,  0, -1,   // from 01
        -1,  0,  0,  1,   // from 10
         0,  1, -1,  0    // from 11
    };

    while (1) {
        if (xQueueReceive(encoder_queue, &new_state, portMAX_DELAY) == pdTRUE) {
            //printf("Last state: %d, new state: %d, which is: %s\n", last_state, new_state, encoder_transition_type(last_state, new_state));
            int8_t delta = state_table[(last_state << 2) | new_state];
            last_state = new_state;

            if (delta != 0) {
                position += delta;
                if (position < 0) position = 0;  // volume knob behavior
                printf("\rEncoder position: %ld", (long)position / 4);
            }
        }
    }
}

void button_task(void *pvParameters) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        printf("Button pressed!\n");
    }

}

void configure_gpio(void) {
    // Configure CLK pin - interrupt on falling edge
    gpio_config_t clk_config = {
        .pin_bit_mask = (1ULL << CLK_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,//GPIO_INTR_ANYEDGE,GPIO_INTR_NEGEDGE
    };
    gpio_config(&clk_config);

    gpio_config_t dt_config = {
        .pin_bit_mask = (1ULL << DT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        // Why this?
        .intr_type = GPIO_INTR_ANYEDGE,//GPIO_INTR_ANYEDGE,GPIO_INTR_NEGEDGE
    };
    gpio_config(&dt_config);

    gpio_config_t sw_config = {
        .pin_bit_mask = (1ULL << SW_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&sw_config);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(CLK_PIN, encoderISR, NULL);
    gpio_isr_handler_add(DT_PIN, encoderISR, NULL);
    gpio_isr_handler_add(SW_PIN, buttonISR, NULL);
}