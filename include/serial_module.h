#ifndef SERIAL_MODULE_H
#define SERIAL_MODULE_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "hardware_hal.h"

#define SERIAL_MAX_MESSAGE_SIZE 128

typedef struct {
    UBaseType_t priority;
    uint16_t stack_size;
    size_t log_queue_depth;
} serial_task_config_t;

typedef struct {
    char message[SERIAL_MAX_MESSAGE_SIZE];
} serial_log_message_t;

bool serial_module_start(const serial_task_config_t& config);
bool serial_module_log(const char* message);
bool serial_module_log_from_isr(const char* message);

#endif // SERIAL_MODULE_H
