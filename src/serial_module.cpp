#include "serial_module.h"

#include "controller_task.h"
#include "potentiometer_task.h"
#include "stepper_task.h"
#include "system_config.h"

#include <cstring>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

static TaskHandle_t serial_task_handle = nullptr;
static QueueHandle_t serial_log_queue = nullptr;
static serial_task_config_t serial_config = {
    .priority = SERIAL_TASK_PRIORITY,
    .stack_size = SERIAL_TASK_STACK_SIZE,
    .log_queue_depth = SERIAL_LOG_QUEUE_LENGTH,
};

static QueueHandle_t cached_stepper_event_queue = nullptr;

static int tokenize_command(const String& command, String tokens[], int max_tokens) {
    int count = 0;
    int length = command.length();
    int index = 0;

    while (index < length && count < max_tokens) {
        while (index < length && isspace(static_cast<unsigned char>(command[index]))) {
            index++;
        }
        if (index >= length) {
            break;
        }
        int start = index;
        while (index < length && !isspace(static_cast<unsigned char>(command[index]))) {
            index++;
        }
        tokens[count++] = command.substring(start, index);
    }

    return count;
}

static const char* pot_name(potentiometer_id_t pot_id) {
    switch (pot_id) {
        case POTENTIOMETER_BASE:
            return "BASE";
        case POTENTIOMETER_ARM:
            return "BRACO";
        default:
            return "?";
    }
}

static bool token_to_pot_id(const String& token, potentiometer_id_t* out_id) {
    if (out_id == nullptr || token.length() == 0) {
        return false;
    }

    char axis = token[0];
    if (axis == 'X' || token.startsWith("BASE")) {
        *out_id = POTENTIOMETER_BASE;
        return true;
    }
    if (axis == 'Y' || token.startsWith("BRACO") || token.startsWith("ARM")) {
        *out_id = POTENTIOMETER_ARM;
        return true;
    }

    return false;
}

static bool parse_axis_float_value(const String& line, char axis, bool* has_value, float* result) {
    int index = line.indexOf(axis);
    if (index < 0) {
        *has_value = false;
        *result = 0.0f;
        return true;
    }

    index++;
    int start = index;
    if (index < line.length() && (line[index] == '+' || line[index] == '-')) {
        index++;
    }

    bool has_digit = false;
    while (index < line.length()) {
        char ch = line[index];
        if (isdigit(ch) || ch == '.') {
            index++;
            has_digit = true;
        } else {
            break;
        }
    }

    if (!has_digit) {
        return false;
    }

    String number = line.substring(start, index);
    *result = number.toFloat();
    *has_value = true;
    return true;
}

static void handle_pot_calibration_command(const String& command, bool set_min) {
    String tokens[3];
    int count = tokenize_command(command, tokens, 3);
    if (count < 2) {
        HAL_SERIAL_PORT.println("Uso: POTMIN <X|Y> ou POTMAX <X|Y>");
        return;
    }

    potentiometer_id_t pot_id;
    if (!token_to_pot_id(tokens[1], &pot_id)) {
        HAL_SERIAL_PORT.println("Eixo invalido. Use X (BASE) ou Y (BRACO).");
        return;
    }

    bool ok = set_min ? potentiometer_set_min(pot_id) : potentiometer_set_max(pot_id);
    if (!ok) {
        HAL_SERIAL_PORT.println("Falha ao calibrar potenciometro (task desabilitada?).");
        return;
    }

    float percent = 0.0f;
    if (!potentiometer_get_percent(pot_id, &percent)) {
        percent = 0.0f;
    }

    char buffer[80];
    snprintf(buffer, sizeof(buffer), "Pot %s %s definido. Atual: %.1f%%",
             pot_name(pot_id), set_min ? "MIN" : "MAX", percent);
    HAL_SERIAL_PORT.println(buffer);
}

static void handle_pot_echo_command(const String& command) {
    String tokens[4];
    int count = tokenize_command(command, tokens, 4);
    if (count < 3) {
        HAL_SERIAL_PORT.println("Uso: POTECHO <X|Y|ALL> <ON|OFF>");
        return;
    }

    bool enable = false;
    if (tokens[2] == "ON") {
        enable = true;
    } else if (tokens[2] == "OFF") {
        enable = false;
    } else {
        HAL_SERIAL_PORT.println("Estado invalido. Use ON ou OFF.");
        return;
    }

    if (tokens[1] == "ALL") {
        bool ok_base = potentiometer_set_echo(POTENTIOMETER_BASE, enable);
        bool ok_arm = potentiometer_set_echo(POTENTIOMETER_ARM, enable);
        if (!ok_base && !ok_arm) {
            HAL_SERIAL_PORT.println("Falha ao configurar echo (tasks desabilitadas?).");
            return;
        }

        HAL_SERIAL_PORT.print("Echo ");
        HAL_SERIAL_PORT.print(enable ? "ativado" : "desativado");
        HAL_SERIAL_PORT.println(" para todos os pots.");
        return;
    }

    potentiometer_id_t pot_id;
    if (!token_to_pot_id(tokens[1], &pot_id)) {
        HAL_SERIAL_PORT.println("Eixo invalido. Use X (BASE) ou Y (BRACO).");
        return;
    }

    if (!potentiometer_set_echo(pot_id, enable)) {
        HAL_SERIAL_PORT.println("Falha ao configurar echo (task desabilitada?).");
        return;
    }

    char buffer[64];
    snprintf(buffer, sizeof(buffer), "Echo %s para pot %s",
             enable ? "ativado" : "desativado", pot_name(pot_id));
    HAL_SERIAL_PORT.println(buffer);
}

static void handle_ctrl_enable_command(const String& command) {
    String tokens[3];
    int count = tokenize_command(command, tokens, 3);
    if (count < 2) {
        HAL_SERIAL_PORT.println("Uso: CTRL <ON|OFF>");
        return;
    }

    if (tokens[1] == "ON") {
        controller_set_enabled(true);
        HAL_SERIAL_PORT.println("Controle ligado.");
    } else if (tokens[1] == "OFF") {
        controller_set_enabled(false);
        HAL_SERIAL_PORT.println("Controle desligado.");
    } else {
        HAL_SERIAL_PORT.println("Estado invalido. Use ON ou OFF.");
    }
}

static bool extract_axis_value_from_token(const String& token, char* axis, float* value) {
    if (token.length() < 2) {
        return false;
    }
    *axis = token[0];
    String number = token.substring(1);
    if (number.length() == 0) {
        return false;
    }
    *value = number.toFloat();
    return true;
}

static void handle_ctrl_setpoint_command(const String& command) {
    String tokens[3];
    int count = tokenize_command(command, tokens, 3);
    if (count < 2) {
        HAL_SERIAL_PORT.println("Uso: CTRLSP Xnnn ou CTRLSP Ynnn");
        return;
    }

    char axis = '\0';
    float value = 0.0f;

    if (count == 2) {
        if (!extract_axis_value_from_token(tokens[1], &axis, &value)) {
            HAL_SERIAL_PORT.println("Formato invalido. Use CTRLSP Xnnn");
            return;
        }
    } else {
        axis = tokens[1][0];
        value = tokens[2].toFloat();
    }

    potentiometer_id_t pot_id;
    if (!token_to_pot_id(String(axis), &pot_id)) {
        HAL_SERIAL_PORT.println("Eixo invalido. Use X (BASE) ou Y (BRACO).");
        return;
    }

    if (!controller_set_setpoint(pot_id, value)) {
        HAL_SERIAL_PORT.println("Falha ao definir setpoint.");
        return;
    }

    HAL_SERIAL_PORT.print("Setpoint do eixo ");
    HAL_SERIAL_PORT.print(pot_name(pot_id));
    HAL_SERIAL_PORT.print(" definido para ");
    HAL_SERIAL_PORT.print(value, 1);
    HAL_SERIAL_PORT.println("%");
}

static void handle_ctrl_pose_echo_command(const String& command) {
    String tokens[4];
    int count = tokenize_command(command, tokens, 4);
    if (count < 3) {
        HAL_SERIAL_PORT.println("Uso: CTRLPOSECHO <X|Y|ALL> <ON|OFF>");
        return;
    }

    bool enable = false;
    if (tokens[2] == "ON") {
        enable = true;
    } else if (tokens[2] == "OFF") {
        enable = false;
    } else {
        HAL_SERIAL_PORT.println("Estado invalido. Use ON ou OFF.");
        return;
    }

    if (tokens[1] == "ALL") {
        if (!controller_set_pose_echo_all(enable)) {
            HAL_SERIAL_PORT.println("Falha ao configurar echo.");
            return;
        }
        HAL_SERIAL_PORT.print("Echo de posicao ");
        HAL_SERIAL_PORT.print(enable ? "ativado" : "desativado");
        HAL_SERIAL_PORT.println(" para todos os eixos.");
        return;
    }

    potentiometer_id_t pot_id;
    if (!token_to_pot_id(tokens[1], &pot_id)) {
        HAL_SERIAL_PORT.println("Eixo invalido. Use X (BASE) ou Y (BRACO).");
        return;
    }

    if (!controller_set_pose_echo(pot_id, enable)) {
        HAL_SERIAL_PORT.println("Falha ao configurar echo.");
        return;
    }

    HAL_SERIAL_PORT.print("Echo de posicao ");
    HAL_SERIAL_PORT.print(enable ? "ativado" : "desativado");
    HAL_SERIAL_PORT.print(" para ");
    HAL_SERIAL_PORT.println(pot_name(pot_id));
}

static void handle_ctrl_go_command(const String& command) {
    bool has_x = false;
    bool has_y = false;
    float x_value = 0.0f;
    float y_value = 0.0f;

    if (!parse_axis_float_value(command, 'X', &has_x, &x_value) ||
        !parse_axis_float_value(command, 'Y', &has_y, &y_value)) {
        HAL_SERIAL_PORT.println("Formato invalido. Use CTRLGO Xnnn Ymmm");
        return;
    }

    if (!has_x && !has_y) {
        HAL_SERIAL_PORT.println("Nenhum eixo informado.");
        return;
    }

    if (!controller_set_setpoints(has_x, x_value, has_y, y_value)) {
        HAL_SERIAL_PORT.println("Falha ao definir setpoints.");
        return;
    }

    HAL_SERIAL_PORT.print("Setpoints atualizados.");
    if (has_x) {
        HAL_SERIAL_PORT.print(" X=");
        HAL_SERIAL_PORT.print(x_value, 1);
        HAL_SERIAL_PORT.print("%");
    }
    if (has_y) {
        HAL_SERIAL_PORT.print(" Y=");
        HAL_SERIAL_PORT.print(y_value, 1);
        HAL_SERIAL_PORT.print("%");
    }
    HAL_SERIAL_PORT.println();
}

static void serial_print_banner() {
    HAL_SERIAL_PORT.println();
    HAL_SERIAL_PORT.println("=================================");
    HAL_SERIAL_PORT.println("     ESP32 FreeRTOS Controller    ");
    HAL_SERIAL_PORT.println("=================================");
    HAL_SERIAL_PORT.println("Comandos:");
    HAL_SERIAL_PORT.println("  HELP      - Lista comandos");
    HAL_SERIAL_PORT.println("  STATUS    - Mostra status");
    HAL_SERIAL_PORT.println("  G01 Xnnn Ymmm");
    HAL_SERIAL_PORT.println("     X -> motor base (passos)");
    HAL_SERIAL_PORT.println("     Y -> motor braco (passos)");
    HAL_SERIAL_PORT.println("     Use + ou - para sentido");
    HAL_SERIAL_PORT.println("  POTMIN X|Y  - define posicao atual como minimo");
    HAL_SERIAL_PORT.println("  POTMAX X|Y  - define posicao atual como maximo");
    HAL_SERIAL_PORT.println("  POTECHO X|Y|ALL <ON|OFF> - echo percentual");
    HAL_SERIAL_PORT.println("  CTRL <ON|OFF> - liga/desliga controle automatico");
    HAL_SERIAL_PORT.println("  CTRLSP Xnnn / Ynnn - define setpoint (%)");
    HAL_SERIAL_PORT.println("  CTRLGO Xnnn Ymmm - define SP de ambos os eixos");
    HAL_SERIAL_PORT.println("  CTRLPOSECHO X|Y|ALL <ON|OFF> - echo da posicao");
    HAL_SERIAL_PORT.println("=================================");
    HAL_SERIAL_PORT.print(SERIAL_COMMAND_PROMPT);
}

static bool parse_axis_value(const String& line, char axis, bool* has_value, int32_t* result) {
    int index = line.indexOf(axis);
    if (index < 0) {
        *has_value = false;
        *result = 0;
        return true;
    }

    int sign = 1;
    index++;

    if (index < line.length()) {
        if (line[index] == '+') {
            sign = 1;
            index++;
        } else if (line[index] == '-') {
            sign = -1;
            index++;
        }
    }

    int start = index;
    while (index < line.length() && isdigit(line[index])) {
        index++;
    }

    if (start == index) {
        return false;
    }

    String number = line.substring(start, index);
    *result = sign * number.toInt();
    *has_value = true;
    return true;
}

static bool enqueue_stepper_move(stepper_motor_id_t motor_id, int32_t signed_steps) {
    if (signed_steps == 0) {
        return true;
    }

    stepper_command_t command;
    command.steps = static_cast<uint32_t>(abs(signed_steps));
    command.direction = signed_steps >= 0 ? STEPPER_DIRECTION_CW : STEPPER_DIRECTION_CCW;
    command.use_ramp = STEPPER_DEFAULT_USE_RAMP;
    command.release_after_move = STEPPER_DEFAULT_RELEASE;
    command.notify_controller = true;

    if (!stepper_submit_command(motor_id, command, pdMS_TO_TICKS(100))) {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "Fila cheia para motor %d", motor_id);
        serial_module_log(buffer);
        return false;
    }

    return true;
}

static void process_g01(const String& line) {
    bool has_x = false;
    bool has_y = false;
    int32_t x_steps = 0;
    int32_t y_steps = 0;

    if (!parse_axis_value(line, 'X', &has_x, &x_steps) ||
        !parse_axis_value(line, 'Y', &has_y, &y_steps)) {
        HAL_SERIAL_PORT.println("Erro: formato invalido. Use G01 Xnnn Ymmm");
        return;
    }

    if (!has_x && !has_y) {
        HAL_SERIAL_PORT.println("Nenhum eixo informado.");
        return;
    }

    bool ok = true;
    if (has_x) {
        ok &= enqueue_stepper_move(STEPPER_MOTOR_BASE, x_steps);
    }
    if (has_y) {
        ok &= enqueue_stepper_move(STEPPER_MOTOR_ARM, y_steps);
    }

    if (ok) {
        HAL_SERIAL_PORT.println("Movimento enfileirado.");
    } else {
        HAL_SERIAL_PORT.println("Falha ao enfileirar movimento.");
    }
}

static void process_command(String command) {
    command.trim();
    command.toUpperCase();

    if (command.isEmpty()) {
        HAL_SERIAL_PORT.print(SERIAL_COMMAND_PROMPT);
        return;
    }

    if (command.startsWith("G01")) {
        process_g01(command);
    } else if (command == "HELP") {
        serial_print_banner();
        return;
    } else if (command == "STATUS") {
        HAL_SERIAL_PORT.print("Heap livre: ");
        HAL_SERIAL_PORT.print(ESP.getFreeHeap());
        HAL_SERIAL_PORT.println(" bytes");
        HAL_SERIAL_PORT.print("Uptime (s): ");
        HAL_SERIAL_PORT.println(millis() / 1000);
    } else if (command.startsWith("POTMIN")) {
        handle_pot_calibration_command(command, true);
    } else if (command.startsWith("POTMAX")) {
        handle_pot_calibration_command(command, false);
    } else if (command.startsWith("POTECHO")) {
        handle_pot_echo_command(command);
    } else if (command.startsWith("CTRLGO")) {
        handle_ctrl_go_command(command);
    } else if (command.startsWith("CTRLSP")) {
        handle_ctrl_setpoint_command(command);
    } else if (command.startsWith("CTRLPOSECHO")) {
        handle_ctrl_pose_echo_command(command);
    } else if (command.startsWith("CTRL")) {
        handle_ctrl_enable_command(command);
    } else {
        HAL_SERIAL_PORT.print("Comando desconhecido: ");
        HAL_SERIAL_PORT.println(command);
    }

    HAL_SERIAL_PORT.print(SERIAL_COMMAND_PROMPT);
}

static void handle_stepper_events() {
    if (cached_stepper_event_queue == nullptr) {
        cached_stepper_event_queue = stepper_event_queue();
    }

    if (cached_stepper_event_queue == nullptr) {
        return;
    }

    stepper_event_t event;
    while (xQueueReceive(cached_stepper_event_queue, &event, 0) == pdPASS) {
        const char* motor_name = event.motor_id == STEPPER_MOTOR_BASE ? "BASE" : "BRACO";
        HAL_SERIAL_PORT.print("Stepper ");
        HAL_SERIAL_PORT.print(motor_name);
        HAL_SERIAL_PORT.print(" concluido: ");
        HAL_SERIAL_PORT.print(event.command.steps);
        HAL_SERIAL_PORT.println(" passos.");
    }
}

static void serial_task(void* pvParameters) {
    (void) pvParameters;

    HAL_SERIAL_PORT.begin(
        HAL_SERIAL_BAUD_RATE,
        HAL_SERIAL_CONFIG,
        HAL_SERIAL_RX_PIN,
        HAL_SERIAL_TX_PIN);

    vTaskDelay(pdMS_TO_TICKS(100));
    serial_print_banner();

    String rx_buffer;
    rx_buffer.reserve(64);
    bool last_char_was_cr = false;

    while (true) {
        handle_stepper_events();

        serial_log_message_t log_message;
        if (serial_log_queue != nullptr &&
            xQueueReceive(serial_log_queue, &log_message, pdMS_TO_TICKS(10)) == pdPASS) {
            HAL_SERIAL_PORT.println(log_message.message);
        }

        while (HAL_SERIAL_PORT.available()) {
            char ch = static_cast<char>(HAL_SERIAL_PORT.read());

            if (ch == '\r' || ch == '\n') {
                if (ch == '\n' && last_char_was_cr) {
                    last_char_was_cr = false;
                    continue;
                }
                last_char_was_cr = (ch == '\r');
                HAL_SERIAL_PORT.print("\r\n");
                process_command(rx_buffer);
                rx_buffer = "";
                continue;
            }

            if (ch == 0x08 || ch == 0x7F) {  // backspace/delete
                last_char_was_cr = false;
                if (rx_buffer.length() > 0) {
                    rx_buffer.remove(rx_buffer.length() - 1);
                    HAL_SERIAL_PORT.print("\b \b");
                }
                continue;
            }

            if (isprint(static_cast<unsigned char>(ch))) {
                last_char_was_cr = false;
                HAL_SERIAL_PORT.write(ch);
                rx_buffer += ch;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool serial_module_start(const serial_task_config_t& config) {
    if (serial_task_handle != nullptr) {
        return true;
    }

    serial_config = config;
    serial_log_queue = xQueueCreate(serial_config.log_queue_depth, sizeof(serial_log_message_t));
    if (serial_log_queue == nullptr) {
        return false;
    }

    BaseType_t result = xTaskCreate(
        serial_task,
        "SerialTask",
        serial_config.stack_size,
        nullptr,
        serial_config.priority,
        &serial_task_handle);

    return result == pdPASS;
}

bool serial_module_log(const char* message) {
    if (serial_log_queue == nullptr || message == nullptr) {
        return false;
    }

    serial_log_message_t log_message;
    strncpy(log_message.message, message, SERIAL_MAX_MESSAGE_SIZE - 1);
    log_message.message[SERIAL_MAX_MESSAGE_SIZE - 1] = '\0';
    return xQueueSend(serial_log_queue, &log_message, 0) == pdPASS;
}

bool serial_module_log_from_isr(const char* message) {
    if (serial_log_queue == nullptr || message == nullptr) {
        return false;
    }

    serial_log_message_t log_message;
    strncpy(log_message.message, message, SERIAL_MAX_MESSAGE_SIZE - 1);
    log_message.message[SERIAL_MAX_MESSAGE_SIZE - 1] = '\0';

    BaseType_t higher_priority_task_woken = pdFALSE;
    BaseType_t status = xQueueSendFromISR(serial_log_queue, &log_message, &higher_priority_task_woken);
    if (higher_priority_task_woken) {
        portYIELD_FROM_ISR();
    }
    return status == pdPASS;
}
