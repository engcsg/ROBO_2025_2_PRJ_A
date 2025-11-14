#ifndef BLINK_MODULE_H
#define BLINK_MODULE_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hardware_hal.h"

#define BLINK_DEFAULT_PERIOD_MS 1000

typedef struct {
    uint32_t period_ms;
    UBaseType_t priority;
    uint16_t stack_size;
} blink_task_config_t;

bool blink_module_start(const blink_task_config_t& config);
void blink_module_stop(void);
void blink_module_set_enabled(bool enabled);
bool blink_module_is_running(void);

#endif // BLINK_MODULE_H
