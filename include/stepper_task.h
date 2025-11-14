#ifndef STEPPER_TASK_H
#define STEPPER_TASK_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "hardware_hal.h"

typedef enum {
    STEPPER_MOTOR_BASE = 0,
    STEPPER_MOTOR_ARM,
    STEPPER_MOTOR_COUNT
} stepper_motor_id_t;

typedef enum {
    STEPPER_DIRECTION_CW = 0,
    STEPPER_DIRECTION_CCW
} stepper_direction_t;

typedef struct {
    uint32_t steps;
    stepper_direction_t direction;
    bool use_ramp;
    bool release_after_move;
    bool notify_controller;
} stepper_command_t;

typedef struct {
    stepper_motor_id_t motor_id;
    gpio_num_t step_pin;
    gpio_num_t dir_pin;
    gpio_num_t enable_pin;
    bool enable_active_low;
    bool invert_direction;
    const char* task_name;
    UBaseType_t priority;
    uint16_t stack_size;
    size_t queue_depth;
    uint32_t pulse_width_us;
    uint32_t step_delay_us;
} stepper_task_config_t;

typedef struct {
    stepper_motor_id_t motor_id;
    stepper_command_t command;
    bool completed;
} stepper_event_t;

bool stepper_module_start(void);
bool stepper_submit_command(stepper_motor_id_t motor_id,
                            const stepper_command_t& command,
                            TickType_t timeout_ticks);
QueueHandle_t stepper_event_queue(void);

#endif // STEPPER_TASK_H
