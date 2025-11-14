#include "potentiometer_task.h"

#include "serial_module.h"
#include "system_config.h"

#include <driver/adc.h>
#include <math.h>

typedef struct {
    potentiometer_task_config_t config;
    TaskHandle_t task_handle;
    bool enabled;
    bool echo_enabled;
    bool report_pending;
    int current_raw;
    float current_percent;
    int min_raw;
    int max_raw;
    float last_report_percent;
    TickType_t last_report_tick;
} potentiometer_context_t;

static potentiometer_context_t pot_contexts[POTENTIOMETER_COUNT] = {};
static bool pot_module_started = false;
static portMUX_TYPE pot_lock = portMUX_INITIALIZER_UNLOCKED;

static const potentiometer_task_config_t default_pot_configs[POTENTIOMETER_COUNT] = {
    {
        .pot_id = POTENTIOMETER_BASE,
        .pin = HAL_POT_BASE_PIN,
        .name = "BASE",
        .priority = POT_TASK_PRIORITY,
        .stack_size = POT_TASK_STACK_SIZE,
        .sample_period_ms = POT_SAMPLE_DELAY_MS,
    },
    {
        .pot_id = POTENTIOMETER_ARM,
        .pin = HAL_POT_ARM_PIN,
        .name = "BRACO",
        .priority = POT_TASK_PRIORITY,
        .stack_size = POT_TASK_STACK_SIZE,
        .sample_period_ms = POT_SAMPLE_DELAY_MS,
    },
};

static inline int pin_number(gpio_num_t pin) {
    return static_cast<int>(pin);
}

static float calculate_percent(int raw_value, int min_raw, int max_raw) {
    if (max_raw <= min_raw + 5) {
        return 0.0f;
    }

    int clamped = raw_value;
    if (clamped < min_raw) clamped = min_raw;
    if (clamped > max_raw) clamped = max_raw;

    return (static_cast<float>(clamped - min_raw) * 100.0f) /
           static_cast<float>(max_raw - min_raw);
}

static void potentiometer_task(void* pvParameters) {
    auto* context = static_cast<potentiometer_context_t*>(pvParameters);
    const TickType_t delay_ticks = pdMS_TO_TICKS(context->config.sample_period_ms);

    char start_msg[48];
    snprintf(start_msg, sizeof(start_msg), "Pot %s task iniciada", context->config.name);
    serial_module_log(start_msg);

    for (;;) {
        int raw_value = analogRead(pin_number(context->config.pin));

        float percent = 0.0f;
        bool echo_enabled = false;
        bool report_pending = false;
        float last_report_percent = 0.0f;
        TickType_t last_report_tick = 0;
        portENTER_CRITICAL(&pot_lock);
        context->current_raw = raw_value;
        percent = calculate_percent(raw_value, context->min_raw, context->max_raw);
        context->current_percent = percent;
        echo_enabled = context->echo_enabled;
        report_pending = context->report_pending;
        last_report_percent = context->last_report_percent;
        last_report_tick = context->last_report_tick;
        portEXIT_CRITICAL(&pot_lock);

        if (echo_enabled) {
            TickType_t now = xTaskGetTickCount();
            bool value_changed = fabsf(percent - last_report_percent) >= POT_ECHO_MIN_DELTA_PERCENT;
            bool enough_time = (now - last_report_tick) >= pdMS_TO_TICKS(POT_ECHO_MIN_INTERVAL_MS);
            bool should_report = report_pending || (value_changed && enough_time);
            if (should_report) {
                char msg[64];
                snprintf(msg, sizeof(msg), "Pot %s: %.1f%%", context->config.name, percent);
                serial_module_log(msg);
                portENTER_CRITICAL(&pot_lock);
                context->report_pending = false;
                context->last_report_percent = percent;
                context->last_report_tick = now;
                portEXIT_CRITICAL(&pot_lock);
            }
        }

        vTaskDelay(delay_ticks);
    }
}

static bool initialize_context(potentiometer_context_t& context,
                               const potentiometer_task_config_t& config) {
    context.config = config;
    context.enabled = true;
    context.echo_enabled = false;
    context.report_pending = false;
    context.current_raw = 0;
    context.current_percent = 0.0f;
    context.min_raw = HAL_POT_ADC_MIN_VALUE;
    context.max_raw = HAL_POT_ADC_MAX_VALUE;
    context.last_report_percent = 0.0f;
    context.last_report_tick = 0;

    BaseType_t result = xTaskCreate(
        potentiometer_task,
        config.name,
        config.stack_size,
        &context,
        config.priority,
        &context.task_handle);

    return result == pdPASS;
}

bool potentiometer_module_start(void) {
    if (pot_module_started) {
        return true;
    }

    const bool pot_enabled[POTENTIOMETER_COUNT] = {
        ENABLE_TASK_POT_BASE,
        ENABLE_TASK_POT_ARM,
    };

    bool any_enabled = false;
    for (size_t i = 0; i < POTENTIOMETER_COUNT; ++i) {
        if (pot_enabled[i]) {
            any_enabled = true;
            break;
        }
    }

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    if (!any_enabled) {
        pot_module_started = true;
        return true;
    }

    for (size_t i = 0; i < POTENTIOMETER_COUNT; ++i) {
        pot_contexts[i].enabled = pot_enabled[i];
        if (!pot_enabled[i]) {
            continue;
        }

        if (!initialize_context(pot_contexts[i], default_pot_configs[i])) {
            return false;
        }
    }

    pot_module_started = true;
    return true;
}

static bool calibrate_with_current(potentiometer_id_t pot_id, bool set_min) {
    if (pot_id < 0 || pot_id >= POTENTIOMETER_COUNT) {
        return false;
    }

    auto& context = pot_contexts[pot_id];
    if (!context.enabled) {
        return false;
    }

    int raw_value = analogRead(pin_number(context.config.pin));

    portENTER_CRITICAL(&pot_lock);
    if (set_min) {
        context.min_raw = raw_value;
        if (context.max_raw <= context.min_raw + 5) {
            context.max_raw = context.min_raw + 5;
        }
    } else {
        context.max_raw = raw_value;
        if (context.max_raw <= context.min_raw + 5) {
            context.min_raw = context.max_raw - 5;
        }
    }
    portEXIT_CRITICAL(&pot_lock);

    return true;
}

bool potentiometer_set_min(potentiometer_id_t pot_id) {
    return calibrate_with_current(pot_id, true);
}

bool potentiometer_set_max(potentiometer_id_t pot_id) {
    return calibrate_with_current(pot_id, false);
}

bool potentiometer_set_echo(potentiometer_id_t pot_id, bool enabled) {
    if (pot_id < 0 || pot_id >= POTENTIOMETER_COUNT) {
        return false;
    }

    auto& context = pot_contexts[pot_id];
    if (!context.enabled) {
        return false;
    }

    portENTER_CRITICAL(&pot_lock);
    context.echo_enabled = enabled;
    if (enabled) {
        context.report_pending = true;
    } else {
        context.report_pending = false;
    }
    portEXIT_CRITICAL(&pot_lock);
    return true;
}

bool potentiometer_get_percent(potentiometer_id_t pot_id, float* out_percent) {
    if (out_percent == nullptr || pot_id < 0 || pot_id >= POTENTIOMETER_COUNT) {
        return false;
    }

    auto& context = pot_contexts[pot_id];
    if (!context.enabled) {
        return false;
    }

    portENTER_CRITICAL(&pot_lock);
    *out_percent = context.current_percent;
    portEXIT_CRITICAL(&pot_lock);
    return true;
}
