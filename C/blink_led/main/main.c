#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#define BLINK_GPIO 8 // MI pin on QT Py C3
#define SWITCH_GPIO 4  // A0 pin on QT Py C3
#define GPIO_TASK_PRIORITY 10

static const char *TAG = "gpio_task";

static QueueHandle_t gpio_evt_queue = NULL;

// ISR - runs when button state changes
static void IRAM_ATTR my_gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Task that processes button events
static void my_gpio_task_dequeuer(void* arg)
{
    uint32_t gpio_num;
    while (1) {
        if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY)) {
            int switch_pressed = (gpio_get_level(SWITCH_GPIO) == 0);
            esp_err_t ret = gpio_set_level(BLINK_GPIO, switch_pressed);

            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to set GPIO level: %s", esp_err_to_name(ret));
            }
        }
    }
}

void app_main(void)
{
    // Configure LED output
    ESP_ERROR_CHECK(gpio_reset_pin(BLINK_GPIO));
    ESP_ERROR_CHECK(gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT));

    // Configure switch input with interrupt
    ESP_ERROR_CHECK(gpio_reset_pin(SWITCH_GPIO));
    ESP_ERROR_CHECK(gpio_set_direction(SWITCH_GPIO, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(SWITCH_GPIO, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_set_intr_type(SWITCH_GPIO, GPIO_INTR_ANYEDGE));

    // Create queue and task
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    if (gpio_evt_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queue");
        abort();
    }
    if (xTaskCreate(my_gpio_task_dequeuer, "my_gpio_task_dequeuer", 2048, NULL, GPIO_TASK_PRIORITY, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GPIO task");
        abort();
    }

    // Install ISR service and attach handler
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(SWITCH_GPIO, my_gpio_isr_handler, (void*) SWITCH_GPIO));
}
