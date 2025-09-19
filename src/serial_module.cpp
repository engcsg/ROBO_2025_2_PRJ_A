#include "serial_module.h"
#include "blink_module.h"

// Variáveis privadas do módulo
static TaskHandle_t serial_task_handle = NULL;
static QueueHandle_t serial_queue = NULL;

// Task para processar mensagens seriais
static void serial_task(void *pvParameters) {
    (void) pvParameters;
    
    serial_message_t message;
    
    // Inicializar comunicação serial
    Serial.begin(115200);
    
    // Aguardar um pouco para estabilizar
    vTaskDelay(pdMS_TO_TICKS(100));
    
    Serial.println("=================================");
    Serial.println("Sistema de Módulos FreeRTOS");
    Serial.println("=================================");
    Serial.println("Serial module initialized");
    Serial.println("Commands:");
    Serial.println("  'on' - Enable blink");
    Serial.println("  'off' - Disable blink");
    Serial.println("  'fast' - Fast blink (200ms)");
    Serial.println("  'slow' - Slow blink (1000ms)");
    Serial.println("  'status' - Show system status");
    Serial.println("=================================");
    
    while (1) {
        // Processar mensagens da fila
        if (xQueueReceive(serial_queue, &message, pdMS_TO_TICKS(10)) == pdPASS) {
            Serial.print("[");
            Serial.print(millis());
            Serial.print("] ");
            Serial.println(message.message);
        }
        
        // Processar comandos recebidos via serial
        if (Serial.available()) {
            String command = Serial.readStringUntil('\n');
            command.trim();
            command.toLowerCase();
            
            Serial.print("Received command: ");
            Serial.println(command);
            
            // Processar comandos (aqui você pode adicionar mais comandos)
            if (command == "status") {
                Serial.println("System Status:");
                Serial.print("Free heap: ");
                Serial.print(ESP.getFreeHeap());
                Serial.println(" bytes");
                Serial.print("Uptime: ");
                Serial.print(millis() / 1000);
                Serial.println(" seconds");
            } else if (command == "help") {
                Serial.println("Available commands:");
                Serial.println("  'on', 'off', 'fast', 'slow', 'status', 'help'");
            } else if (command == "on") {
                blink_enable(true);
            } else if (command == "off") {
                blink_enable(false);
            } else if (command == "fast") {
                blink_config_t fast_config = {200, 200, true};
                blink_set_config(fast_config);
            } else if (command == "slow") {
                blink_config_t slow_config = {1000, 1000, true};
                blink_set_config(slow_config);
            } else {
                // Ecoar comando não reconhecido
                String response = "Unknown command: " + command;
                Serial.println(response);
            }
        }
        
        // Pequeno delay para não sobrecarregar a CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Inicializar o módulo serial
void serial_module_init(void) {
    // Criar fila para mensagens
    serial_queue = xQueueCreate(SERIAL_QUEUE_SIZE, sizeof(serial_message_t));
    
    if (serial_queue == NULL) {
        Serial.println("ERROR: Failed to create serial queue");
        return;
    }
    
    // Criar task serial
    BaseType_t result = xTaskCreate(
        serial_task,                // Função da task
        "SerialTask",               // Nome da task
        SERIAL_TASK_STACK_SIZE,     // Tamanho da stack
        NULL,                       // Parâmetros
        SERIAL_TASK_PRIORITY,       // Prioridade
        &serial_task_handle         // Handle da task
    );
    
    if (result != pdPASS) {
        Serial.println("ERROR: Failed to create serial task");
    }
}

// Enviar mensagem via serial (de outras tasks)
bool serial_send_message(const char* message) {
    if (serial_queue == NULL) {
        return false;
    }
    
    serial_message_t msg;
    strncpy(msg.message, message, MAX_MESSAGE_SIZE - 1);
    msg.message[MAX_MESSAGE_SIZE - 1] = '\0';
    msg.length = strlen(msg.message);
    
    return xQueueSend(serial_queue, &msg, pdMS_TO_TICKS(100)) == pdPASS;
}

// Enviar mensagem via serial (de ISR)
bool serial_send_message_from_isr(const char* message) {
    if (serial_queue == NULL) {
        return false;
    }
    
    serial_message_t msg;
    strncpy(msg.message, message, MAX_MESSAGE_SIZE - 1);
    msg.message[MAX_MESSAGE_SIZE - 1] = '\0';
    msg.length = strlen(msg.message);
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BaseType_t result = xQueueSendFromISR(serial_queue, &msg, &xHigherPriorityTaskWoken);
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
    
    return result == pdPASS;
}