#include <Arduino.h>

#include "blink_module.h"
#include "hardware_hal.h"
#include "serial_module.h"
#include "stepper_task.h"
#include "system_config.h"

static void start_serial() {
    if (!ENABLE_TASK_SERIAL) {
        HAL_SERIAL_PORT.begin(HAL_SERIAL_BAUD_RATE, HAL_SERIAL_CONFIG, HAL_SERIAL_RX_PIN, HAL_SERIAL_TX_PIN);
        return;
    }

    serial_task_config_t config = {
        .priority = SERIAL_TASK_PRIORITY,
        .stack_size = SERIAL_TASK_STACK_SIZE,
        .log_queue_depth = SERIAL_LOG_QUEUE_LENGTH,
    };
    serial_module_start(config);
}

static void start_blink() {
    if (!ENABLE_TASK_BLINK) {
        return;
    }

    blink_task_config_t config = {
        .period_ms = BLINK_DEFAULT_PERIOD_MS,
        .priority = BLINK_TASK_PRIORITY,
        .stack_size = BLINK_TASK_STACK_SIZE,
    };
    blink_module_start(config);
}

static void start_steppers() {
    if (!ENABLE_TASK_STEPPER_BASE && !ENABLE_TASK_STEPPER_ARM) {
        return;
    }
    stepper_module_start();
}

void setup() {
    start_serial();
    vTaskDelay(pdMS_TO_TICKS(100));

    serial_module_log("Inicializando tarefas...");

    start_blink();
    start_steppers();

    serial_module_log("Sistema pronto.");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(100));
}
