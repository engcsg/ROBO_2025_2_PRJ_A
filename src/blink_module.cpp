#include "blink_module.h"
#include "serial_module.h"

// Variáveis privadas do módulo
static TaskHandle_t blink_task_handle = NULL;
static blink_config_t current_config = {500, 500, true}; // Configuração padrão

// Task principal do módulo de blink
static void blink_task(void *pvParameters) {
    (void) pvParameters;
    
    // Configurar o pino do LED
    pinMode(HAL_LED_BUILTIN_PIN, OUTPUT);
    
    // Enviar mensagem de inicialização
    serial_send_message("Blink module initialized");
    
    while (1) {
        if (current_config.enabled) {
            // Ligar LED
            digitalWrite(HAL_LED_BUILTIN_PIN, HIGH);
            serial_send_message("LED ON");
            vTaskDelay(pdMS_TO_TICKS(current_config.on_time_ms));
            
            // Desligar LED
            digitalWrite(HAL_LED_BUILTIN_PIN, LOW);
            serial_send_message("LED OFF");
            vTaskDelay(pdMS_TO_TICKS(current_config.off_time_ms));
        } else {
            // Se desabilitado, manter LED desligado e verificar a cada 100ms
            digitalWrite(HAL_LED_BUILTIN_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// Inicializar o módulo de blink
void blink_module_init(void) {
    BaseType_t result = xTaskCreate(
        blink_task,                 // Função da task
        "BlinkTask",                // Nome da task
        BLINK_TASK_STACK_SIZE,      // Tamanho da stack
        NULL,                       // Parâmetros
        BLINK_TASK_PRIORITY,        // Prioridade
        &blink_task_handle          // Handle da task
    );
    
    if (result != pdPASS) {
        serial_send_message("ERROR: Failed to create blink task");
    }
}

// Configurar parâmetros do blink
void blink_set_config(blink_config_t config) {
    current_config = config;
    
    char msg[64];
    snprintf(msg, sizeof(msg), "Blink config: ON=%dms, OFF=%dms, Enabled=%s", 
             config.on_time_ms, config.off_time_ms, config.enabled ? "YES" : "NO");
    serial_send_message(msg);
}

// Habilitar/desabilitar o blink
void blink_enable(bool enable) {
    current_config.enabled = enable;
    
    char msg[32];
    snprintf(msg, sizeof(msg), "Blink %s", enable ? "ENABLED" : "DISABLED");
    serial_send_message(msg);
}