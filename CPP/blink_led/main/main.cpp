#include <cstdio>
#include <cstdint>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

namespace {

constexpr gpio_num_t kBlinkGpio = GPIO_NUM_8;   // MI pin on QT Py C3
constexpr gpio_num_t kSwitchGpio = GPIO_NUM_4;  // A0 pin on QT Py C3
constexpr UBaseType_t kGpioTaskPriority = 10;
constexpr uint32_t kQueueSize = 10;
constexpr uint32_t kTaskStackSize = 2048;

const char* kTag = "gpio_task";

class GpioPin {
public:
    explicit GpioPin(gpio_num_t pin) : pin_(pin) {
        ESP_ERROR_CHECK(gpio_reset_pin(pin_));
    }

    void configureAsOutput() {
        ESP_ERROR_CHECK(gpio_set_direction(pin_, GPIO_MODE_OUTPUT));
    }

    void configureAsInput(gpio_pull_mode_t pull_mode = GPIO_FLOATING) {
        ESP_ERROR_CHECK(gpio_set_direction(pin_, GPIO_MODE_INPUT));
        ESP_ERROR_CHECK(gpio_set_pull_mode(pin_, pull_mode));
    }

    void enableInterrupt(gpio_int_type_t intr_type) {
        ESP_ERROR_CHECK(gpio_set_intr_type(pin_, intr_type));
    }

    void attachIsrHandler(gpio_isr_t handler) {
        ESP_ERROR_CHECK(gpio_isr_handler_add(pin_, handler, reinterpret_cast<void*>(pin_)));
    }

    [[nodiscard]] int getLevel() const {
        return gpio_get_level(pin_);
    }

    esp_err_t setLevel(int level) {
        return gpio_set_level(pin_, level);
    }

    [[nodiscard]] gpio_num_t pin() const { return pin_; }

private:
    gpio_num_t pin_;
};

class GpioEventQueue {
public:
    explicit GpioEventQueue(uint32_t size)
        : queue_(xQueueCreate(size, sizeof(uint32_t))) {
        if (queue_ == nullptr) {
            ESP_LOGE(kTag, "Failed to create queue");
            abort();
        }
    }

    ~GpioEventQueue() {
        if (queue_ != nullptr) {
            vQueueDelete(queue_);
        }
    }

    GpioEventQueue(const GpioEventQueue&) = delete;
    GpioEventQueue& operator=(const GpioEventQueue&) = delete;

    void sendFromIsr(uint32_t gpio_num, BaseType_t* higher_priority_woken) {
        xQueueSendFromISR(queue_, &gpio_num, higher_priority_woken);
    }

    [[nodiscard]] bool receive(uint32_t& gpio_num, TickType_t timeout = portMAX_DELAY) {
        return xQueueReceive(queue_, &gpio_num, timeout) == pdTRUE;
    }

    [[nodiscard]] QueueHandle_t handle() const { return queue_; }

private:
    QueueHandle_t queue_;
};

// Global instances for ISR access
GpioEventQueue* g_event_queue = nullptr;
GpioPin* g_led_pin = nullptr;
GpioPin* g_switch_pin = nullptr;

// ISR handler - must be IRAM_ATTR and use C-compatible signature
void IRAM_ATTR gpioIsrHandler(void* arg) {
    uint32_t gpio_num = reinterpret_cast<uint32_t>(arg);
    BaseType_t higher_priority_woken = pdFALSE;
    g_event_queue->sendFromIsr(gpio_num, &higher_priority_woken);
    portYIELD_FROM_ISR(higher_priority_woken);
}

// Task function for processing GPIO events
void gpioTaskHandler(void* arg) {
    uint32_t gpio_num;
    while (true) {
        if (g_event_queue->receive(gpio_num)) {
            bool switch_pressed = (g_switch_pin->getLevel() == 0);
            esp_err_t ret = g_led_pin->setLevel(switch_pressed ? 1 : 0);

            if (ret != ESP_OK) {
                ESP_LOGE(kTag, "Failed to set GPIO level: %s", esp_err_to_name(ret));
            }
        }
    }
}

}  // anonymous namespace

extern "C" void app_main(void) {
    // Create GPIO pins with RAII
    static GpioPin led_pin(kBlinkGpio);
    static GpioPin switch_pin(kSwitchGpio);
    static GpioEventQueue event_queue(kQueueSize);

    // Set up global pointers for ISR access
    g_led_pin = &led_pin;
    g_switch_pin = &switch_pin;
    g_event_queue = &event_queue;

    // Configure pins
    led_pin.configureAsOutput();
    switch_pin.configureAsInput(GPIO_PULLUP_ONLY);
    switch_pin.enableInterrupt(GPIO_INTR_ANYEDGE);

    // Create event processing task
    if (xTaskCreate(gpioTaskHandler, "gpio_task", kTaskStackSize,
                    nullptr, kGpioTaskPriority, nullptr) != pdPASS) {
        ESP_LOGE(kTag, "Failed to create GPIO task");
        abort();
    }

    // Install ISR service and attach handler
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    switch_pin.attachIsrHandler(gpioIsrHandler);
}
