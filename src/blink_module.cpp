#include "blink_module.h"
#include "serial_module.h"
#include "system_config.h"

static TaskHandle_t blink_task_handle = nullptr;
static blink_task_config_t blink_config = {
    .period_ms = BLINK_DEFAULT_PERIOD_MS,
    .priority = BLINK_TASK_PRIORITY,
    .stack_size = BLINK_TASK_STACK_SIZE,
};
static bool blink_enabled = true;

static void blink_task(void* pvParameters) {
    (void) pvParameters;

    pinMode(HAL_LED_BUILTIN_PIN, OUTPUT);
    serial_module_log("Blink task running");

    const TickType_t idle_delay = pdMS_TO_TICKS(100);

    while (true) {
        if (!blink_enabled) {
            digitalWrite(HAL_LED_BUILTIN_PIN, LOW);
            vTaskDelay(idle_delay);
            continue;
        }

        digitalWrite(HAL_LED_BUILTIN_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(blink_config.period_ms / 2));
        digitalWrite(HAL_LED_BUILTIN_PIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(blink_config.period_ms / 2));
    }
}

bool blink_module_start(const blink_task_config_t& config) {
    if (blink_task_handle != nullptr) {
        return true;
    }

    blink_config = config;
    if (blink_config.period_ms < 2) {
        blink_config.period_ms = 2;
    }

    BaseType_t result = xTaskCreate(
        blink_task,
        "BlinkTask",
        blink_config.stack_size,
        nullptr,
        blink_config.priority,
        &blink_task_handle);

    if (result != pdPASS) {
        blink_task_handle = nullptr;
        return false;
    }

    return true;
}

void blink_module_stop(void) {
    if (blink_task_handle != nullptr) {
        vTaskDelete(blink_task_handle);
        blink_task_handle = nullptr;
    }

    digitalWrite(HAL_LED_BUILTIN_PIN, LOW);
}

void blink_module_set_enabled(bool enabled) {
    blink_enabled = enabled;
}

bool blink_module_is_running(void) {
    return blink_task_handle != nullptr;
}
