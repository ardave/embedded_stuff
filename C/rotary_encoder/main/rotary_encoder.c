#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
#define BUTTON_DEBOUNCE_TIME_US 50000  // 50ms debounce
#define ENCODER_DEBOUNCE_TIME_US 50000 // 50ms debounce

void encoder_task(void *pvParameters);
void configure_gpio(void);

volatile int32_t encoderDelta = 0;
volatile bool buttonPressed = false;

TaskHandle_t encoderTaskHandle = NULL;

void app_main(void)
{
    configure_gpio();

    xTaskCreate(
        encoder_task,
        "encoder_task",
        2048,
        NULL,
        5,
        &encoderTaskHandle
    );
}

void IRAM_ATTR encoderISR(void *arg) {
    static int64_t lastEdgeTime = 0;
    int64_t now = esp_timer_get_time();
    static uint8_t lastState = 0;
    static const int8_t stateTable[16] = {
        0, -1, 1, 0, // from 00
        1, 0, 0, -1, // from 01
        -1, 0, 0, 1, // from 10
        0, 1, -1, 0  // from 11
    };

    if ((now - lastEdgeTime) > ENCODER_DEBOUNCE_TIME_US) {

        // Read both pins atomically from GPIO input register
        uint32_t gpio_in = REG_READ(GPIO_IN_REG);

        uint8_t clk = (gpio_in >> CLK_PIN) & 1;
        uint8_t dt = (gpio_in >> DT_PIN) & 1;
        uint8_t newState = (clk << 1) | dt;

        int8_t delta = stateTable[(lastState << 2) | newState];
        lastState = newState;

        if (delta != 0) {
            encoderDelta += delta;
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(encoderTaskHandle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        lastEdgeTime = now;
    }
}

void IRAM_ATTR buttonISR(void *arg) {
    static int64_t lastEdgeTime = 0;
    static int lastStableState = 1; // 1 = released (pull-up)
    int64_t now = esp_timer_get_time();

    if ((now - lastEdgeTime) > BUTTON_DEBOUNCE_TIME_US) {
        int currentState = gpio_get_level(SW_PIN);
        // Only trigger on released (1) -> pressed (0) transition
        if (lastStableState == 1 && currentState == 0) {
            buttonPressed = true;
            // Why do we need to notify that a higher priority task was NOT awoken?
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(encoderTaskHandle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        lastStableState = currentState;
        lastEdgeTime = now;
    }
}


void encoder_task(void *pvParameters) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        static int32_t position = 0;

        if (encoderDelta != 0) {
            int32_t delta = encoderDelta;
            encoderDelta = 0;
            position += delta;
            printf("\rEncoder position: %ld", (long)position);
            fflush(stdout);
        }

        if (buttonPressed) {
            buttonPressed = false;
            printf("Button pressed!\n");
        }
    }
}

void configure_gpio(void) {
    // Configure CLK pin - interrupt on falling edge
    gpio_config_t clk_config = {
        .pin_bit_mask = (1ULL << CLK_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&clk_config);

    gpio_config_t dt_config = {
        .pin_bit_mask = (1ULL << DT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        // Why this?
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&dt_config);

    gpio_config_t sw_config = {
        .pin_bit_mask = (1ULL << SW_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&sw_config);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(CLK_PIN, encoderISR, NULL);
    gpio_isr_handler_add(DT_PIN, encoderISR, NULL);
    gpio_isr_handler_add(SW_PIN, buttonISR, NULL);
}