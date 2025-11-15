#include "controller_task.h"

#include "serial_module.h"
#include "system_config.h"

#include <math.h>
#include <stdlib.h>

typedef struct {
    potentiometer_id_t pot_id;
    stepper_motor_id_t motor_id;
    const char* name;
    TaskHandle_t task_handle;
    QueueHandle_t sample_queue;
    float setpoint_percent;
    float current_percent;
    TickType_t last_command_tick;
    bool echo_enabled;
    TickType_t last_echo_tick;
    bool echo_pending;
} controller_context_t;

static controller_context_t controller_contexts[CONTROLLER_AXIS_COUNT] = {
    {
        .pot_id = POTENTIOMETER_BASE,
        .motor_id = STEPPER_MOTOR_BASE,
        .name = "BASE",
        .task_handle = nullptr,
        .sample_queue = nullptr,
        .setpoint_percent = CONTROLLER_DEFAULT_SETPOINT,
        .current_percent = 0.0f,
        .last_command_tick = 0,
        .echo_enabled = false,
        .last_echo_tick = 0,
        .echo_pending = false,
    },
    {
        .pot_id = POTENTIOMETER_ARM,
        .motor_id = STEPPER_MOTOR_ARM,
        .name = "BRACO",
        .task_handle = nullptr,
        .sample_queue = nullptr,
        .setpoint_percent = CONTROLLER_DEFAULT_SETPOINT,
        .current_percent = 0.0f,
        .last_command_tick = 0,
        .echo_enabled = false,
        .last_echo_tick = 0,
        .echo_pending = false,
    },
};

static bool controller_enabled = false;
static bool controller_module_initialized = false;

static bool issue_stepper_command(controller_context_t& context, float error_percent) {
    int32_t steps = static_cast<int32_t>(roundf(error_percent * CONTROLLER_PERCENT_TO_STEPS));
    if (steps == 0) {
        return false;
    }

    if (steps > CONTROLLER_MAX_STEPS_PER_COMMAND) {
        steps = CONTROLLER_MAX_STEPS_PER_COMMAND;
    } else if (steps < -CONTROLLER_MAX_STEPS_PER_COMMAND) {
        steps = -CONTROLLER_MAX_STEPS_PER_COMMAND;
    }

    stepper_command_t command = {
        .steps = static_cast<uint32_t>(abs(steps)),
        .direction = steps >= 0 ? STEPPER_DIRECTION_CW : STEPPER_DIRECTION_CCW,
        .use_ramp = true,
        .release_after_move = false,
        .notify_controller = false,
    };

    return stepper_submit_command(context.motor_id, command, 0);
}

static void emit_echo(controller_context_t& context) {
    char message[64];
    snprintf(message, sizeof(message), "CTRL %s pos: %.1f%% SP: %.1f%%",
             context.name, context.current_percent, context.setpoint_percent);
    serial_module_log(message);
}

static void controller_task(void* pvParameters) {
    auto* context = static_cast<controller_context_t*>(pvParameters);

    char message[64];
    snprintf(message, sizeof(message), "Controller %s iniciado", context->name);
    serial_module_log(message);

    potentiometer_sample_t sample;
    while (true) {
        if (context->sample_queue == nullptr) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (xQueueReceive(context->sample_queue, &sample, pdMS_TO_TICKS(100)) != pdPASS) {
            continue;
        }

        context->current_percent = sample.percent;

        if (context->echo_enabled) {
            TickType_t now = xTaskGetTickCount();
            bool time_elapsed = (now - context->last_echo_tick) >= pdMS_TO_TICKS(CONTROLLER_ECHO_INTERVAL_MS);
            if (context->echo_pending || time_elapsed) {
                emit_echo(*context);
                context->echo_pending = false;
                context->last_echo_tick = now;
            }
        }

        if (!controller_enabled) {
            continue;
        }

        float error = context->setpoint_percent - context->current_percent;
        if (fabsf(error) < CONTROLLER_DEADBAND_PERCENT) {
            continue;
        }

        TickType_t now = xTaskGetTickCount();
        if ((now - context->last_command_tick) < pdMS_TO_TICKS(CONTROLLER_COMMAND_INTERVAL_MS)) {
            continue;
        }

        if (issue_stepper_command(*context, error)) {
            context->last_command_tick = now;
        }
    }
}

bool controller_module_start(void) {
    if (controller_module_initialized) {
        return true;
    }

    const bool axis_enabled[CONTROLLER_AXIS_COUNT] = {
        ENABLE_TASK_CONTROLLER_BASE,
        ENABLE_TASK_CONTROLLER_ARM,
    };

    bool any_enabled = false;
    for (size_t i = 0; i < CONTROLLER_AXIS_COUNT; ++i) {
        if (axis_enabled[i]) {
            any_enabled = true;
        }
    }

    if (!any_enabled) {
        controller_module_initialized = true;
        return true;
    }

    for (size_t i = 0; i < CONTROLLER_AXIS_COUNT; ++i) {
        if (!axis_enabled[i]) {
            continue;
        }

        controller_contexts[i].sample_queue = potentiometer_sample_queue(controller_contexts[i].pot_id);
        if (controller_contexts[i].sample_queue == nullptr) {
            serial_module_log("Controller: fila de pot invalida");
            return false;
        }

        BaseType_t result = xTaskCreate(
            controller_task,
            controller_contexts[i].name,
            CONTROLLER_TASK_STACK_SIZE,
            &controller_contexts[i],
            CONTROLLER_TASK_PRIORITY,
            &controller_contexts[i].task_handle);

        if (result != pdPASS) {
            serial_module_log("Controller: falha ao criar task");
            return false;
        }
    }

    controller_module_initialized = true;
    return true;
}

void controller_set_enabled(bool enabled) {
    controller_enabled = enabled;
    if (enabled) {
        serial_module_log("Controle habilitado");
    } else {
        serial_module_log("Controle desabilitado");
    }
}

bool controller_is_enabled(void) {
    return controller_enabled;
}

static controller_context_t* get_context_from_pot(potentiometer_id_t pot_id) {
    for (size_t i = 0; i < CONTROLLER_AXIS_COUNT; ++i) {
        if (controller_contexts[i].pot_id == pot_id) {
            return &controller_contexts[i];
        }
    }
    return nullptr;
}

static float clamp_percent(float value) {
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 100.0f) {
        return 100.0f;
    }
    return value;
}

bool controller_set_setpoint(potentiometer_id_t pot_id, float percent) {
    controller_context_t* context = get_context_from_pot(pot_id);
    if (context == nullptr) {
        return false;
    }

    context->setpoint_percent = clamp_percent(percent);
    context->echo_pending = true;
    return true;
}

bool controller_set_setpoints(bool has_base, float base_percent,
                              bool has_arm, float arm_percent) {
    bool ok = true;
    if (has_base) {
        ok &= controller_set_setpoint(POTENTIOMETER_BASE, base_percent);
    }
    if (has_arm) {
        ok &= controller_set_setpoint(POTENTIOMETER_ARM, arm_percent);
    }
    return ok;
}

bool controller_set_pose_echo(potentiometer_id_t pot_id, bool enabled) {
    controller_context_t* context = get_context_from_pot(pot_id);
    if (context == nullptr) {
        return false;
    }

    context->echo_enabled = enabled;
    if (enabled) {
        context->echo_pending = true;
    }
    return true;
}

bool controller_set_pose_echo_all(bool enabled) {
    bool ok = true;
    ok &= controller_set_pose_echo(POTENTIOMETER_BASE, enabled);
    ok &= controller_set_pose_echo(POTENTIOMETER_ARM, enabled);
    return ok;
}

bool controller_get_position(potentiometer_id_t pot_id, float* out_percent) {
    if (out_percent == nullptr) {
        return false;
    }

    controller_context_t* context = get_context_from_pot(pot_id);
    if (context == nullptr) {
        return false;
    }

    *out_percent = context->current_percent;
    return true;
}
