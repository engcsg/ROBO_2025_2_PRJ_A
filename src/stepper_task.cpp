#include "stepper_task.h"

#include "serial_module.h"
#include "system_config.h"

#include <stdio.h>

typedef struct {
    stepper_task_config_t config;
    TaskHandle_t handle;
    QueueHandle_t command_queue;
    bool enabled;
} stepper_task_context_t;

static stepper_task_context_t stepper_tasks[STEPPER_MOTOR_COUNT] = {};
static QueueHandle_t completion_queue = nullptr;
static bool module_started = false;

static const stepper_task_config_t default_configs[STEPPER_MOTOR_COUNT] = {
    {
        .motor_id = STEPPER_MOTOR_BASE,
        .step_pin = HAL_STEPPER_BASE_STEP_PIN,
        .dir_pin = HAL_STEPPER_BASE_DIR_PIN,
        .enable_pin = HAL_STEPPER_BASE_ENABLE_PIN,
        .enable_active_low = HAL_STEPPER_BASE_ENABLE_ACTIVE_LOW,
        .invert_direction = false,
        .task_name = "StepperBase",
        .priority = STEPPER_TASK_PRIORITY,
        .stack_size = STEPPER_TASK_STACK_SIZE,
        .queue_depth = STEPPER_QUEUE_LENGTH,
        .pulse_width_us = HAL_STEPPER_DEFAULT_PULSE_WIDTH_US,
        .step_delay_us = HAL_STEPPER_DEFAULT_STEP_DELAY_US,
    },
    {
        .motor_id = STEPPER_MOTOR_ARM,
        .step_pin = HAL_STEPPER_ARM_STEP_PIN,
        .dir_pin = HAL_STEPPER_ARM_DIR_PIN,
        .enable_pin = HAL_STEPPER_ARM_ENABLE_PIN,
        .enable_active_low = HAL_STEPPER_ARM_ENABLE_ACTIVE_LOW,
        .invert_direction = false,
        .task_name = "StepperArm",
        .priority = STEPPER_TASK_PRIORITY,
        .stack_size = STEPPER_TASK_STACK_SIZE,
        .queue_depth = STEPPER_QUEUE_LENGTH,
        .pulse_width_us = HAL_STEPPER_DEFAULT_PULSE_WIDTH_US,
        .step_delay_us = HAL_STEPPER_DEFAULT_STEP_DELAY_US,
    },
};

static uint8_t to_gpio_number(gpio_num_t pin) {
    return static_cast<uint8_t>(pin);
}

static uint8_t level_active(bool active_low) {
    return active_low ? LOW : HIGH;
}

static uint8_t level_inactive(bool active_low) {
    return active_low ? HIGH : LOW;
}

static uint32_t compute_ramped_delay(const stepper_command_t& command,
                                     const stepper_task_config_t& config,
                                     uint32_t current_step) {
    uint32_t delay_us = config.step_delay_us;

    if (!command.use_ramp || command.steps < 4) {
        return delay_us;
    }

    uint32_t ramp_steps = command.steps / 10;
    if (ramp_steps < 2) {
        ramp_steps = 2;
    }
    if (ramp_steps > command.steps / 2) {
        ramp_steps = command.steps / 2;
    }

    if (current_step < ramp_steps) {
        uint32_t delta = ramp_steps - current_step;
        delay_us += (config.step_delay_us / 2) * delta / ramp_steps;
    } else {
        uint32_t remaining = command.steps - current_step;
        if (remaining <= ramp_steps) {
            uint32_t delta = ramp_steps - remaining;
            delay_us += (config.step_delay_us / 2) * delta / ramp_steps;
        }
    }

    return delay_us;
}

static void notify_completion(const stepper_task_context_t& context,
                              const stepper_command_t& command) {
    if (!command.notify_controller || completion_queue == nullptr) {
        return;
    }

    stepper_event_t event = {
        .motor_id = context.config.motor_id,
        .command = command,
        .completed = true,
    };

    xQueueSend(completion_queue, &event, 0);
}

static void execute_command(const stepper_task_context_t& context, const stepper_command_t& command) {
    const auto& cfg = context.config;

    const uint8_t enable_active = level_active(cfg.enable_active_low);
    const uint8_t enable_inactive = level_inactive(cfg.enable_active_low);

    digitalWrite(to_gpio_number(cfg.enable_pin), enable_active);

    uint8_t dir_level = command.direction == STEPPER_DIRECTION_CW ? HIGH : LOW;
    if (cfg.invert_direction) {
        dir_level = dir_level == HIGH ? LOW : HIGH;
    }
    digitalWrite(to_gpio_number(cfg.dir_pin), dir_level);
    digitalWrite(to_gpio_number(cfg.step_pin), LOW);

    for (uint32_t i = 0; i < command.steps; ++i) {
        digitalWrite(to_gpio_number(cfg.step_pin), HIGH);
        delayMicroseconds(cfg.pulse_width_us);
        digitalWrite(to_gpio_number(cfg.step_pin), LOW);

        uint32_t delay_us = compute_ramped_delay(command, cfg, i);
        delayMicroseconds(delay_us);
        taskYIELD();
    }

    if (command.release_after_move) {
        digitalWrite(to_gpio_number(cfg.enable_pin), enable_inactive);
    }

    notify_completion(context, command);
}

static void stepper_task_runner(void* pvParameters) {
    auto* context = static_cast<stepper_task_context_t*>(pvParameters);
    const auto& cfg = context->config;

    pinMode(to_gpio_number(cfg.step_pin), OUTPUT);
    pinMode(to_gpio_number(cfg.dir_pin), OUTPUT);
    pinMode(to_gpio_number(cfg.enable_pin), OUTPUT);
    digitalWrite(to_gpio_number(cfg.step_pin), LOW);
    digitalWrite(to_gpio_number(cfg.dir_pin), LOW);
    digitalWrite(to_gpio_number(cfg.enable_pin), level_inactive(cfg.enable_active_low));

    char message[64];
    snprintf(message, sizeof(message), "%s inicializada", cfg.task_name);
    serial_module_log(message);

    stepper_command_t command;
    while (true) {
        if (xQueueReceive(context->command_queue, &command, portMAX_DELAY) == pdPASS) {
            execute_command(*context, command);
        }
    }
}

QueueHandle_t stepper_event_queue(void) {
    return completion_queue;
}

bool stepper_module_start(void) {
    if (module_started) {
        return true;
    }

    const bool motor_enabled[STEPPER_MOTOR_COUNT] = {
        ENABLE_TASK_STEPPER_BASE,
        ENABLE_TASK_STEPPER_ARM,
    };

    bool any_enabled = false;
    for (size_t i = 0; i < STEPPER_MOTOR_COUNT; ++i) {
        if (motor_enabled[i]) {
            any_enabled = true;
            break;
        }
    }

    if (!any_enabled) {
        module_started = true;
        return true;
    }

    completion_queue = xQueueCreate(STEPPER_COMPLETION_QUEUE, sizeof(stepper_event_t));
    if (completion_queue == nullptr) {
        return false;
    }

    for (size_t i = 0; i < STEPPER_MOTOR_COUNT; ++i) {
        stepper_tasks[i].enabled = motor_enabled[i];
        if (!motor_enabled[i]) {
            continue;
        }

        stepper_tasks[i].config = default_configs[i];
        stepper_tasks[i].command_queue =
            xQueueCreate(stepper_tasks[i].config.queue_depth, sizeof(stepper_command_t));
        if (stepper_tasks[i].command_queue == nullptr) {
            return false;
        }

        BaseType_t result = xTaskCreate(
            stepper_task_runner,
            stepper_tasks[i].config.task_name,
            stepper_tasks[i].config.stack_size,
            &stepper_tasks[i],
            stepper_tasks[i].config.priority,
            &stepper_tasks[i].handle);

        if (result != pdPASS) {
            return false;
        }
    }

    module_started = true;
    return true;
}

bool stepper_submit_command(stepper_motor_id_t motor_id,
                            const stepper_command_t& command,
                            TickType_t timeout_ticks) {
    if (motor_id < 0 || motor_id >= STEPPER_MOTOR_COUNT) {
        return false;
    }

    auto& context = stepper_tasks[motor_id];
    if (!context.enabled || context.command_queue == nullptr) {
        return false;
    }

    return xQueueSend(context.command_queue, &command, timeout_ticks) == pdPASS;
}
