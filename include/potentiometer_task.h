#ifndef POTENTIOMETER_TASK_H
#define POTENTIOMETER_TASK_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "hardware_hal.h"

typedef enum {
    POTENTIOMETER_BASE = 0,
    POTENTIOMETER_ARM,
    POTENTIOMETER_COUNT
} potentiometer_id_t;

typedef struct {
    potentiometer_id_t pot_id;
    gpio_num_t pin;
    const char* name;
    UBaseType_t priority;
    uint16_t stack_size;
    uint32_t sample_period_ms;
} potentiometer_task_config_t;

typedef struct {
    potentiometer_id_t pot_id;
    int raw_value;
    float percent;
    TickType_t timestamp;
} potentiometer_sample_t;

bool potentiometer_module_start(void);
bool potentiometer_set_min(potentiometer_id_t pot_id);
bool potentiometer_set_max(potentiometer_id_t pot_id);
bool potentiometer_set_echo(potentiometer_id_t pot_id, bool enabled);
bool potentiometer_get_percent(potentiometer_id_t pot_id, float* out_percent);
QueueHandle_t potentiometer_sample_queue(potentiometer_id_t pot_id);

#endif  // POTENTIOMETER_TASK_H
