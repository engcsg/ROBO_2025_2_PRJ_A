#ifndef SERIAL_MODULE_H
#define SERIAL_MODULE_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "hardware_hal.h"

// Configurações do módulo serial
#define SERIAL_TASK_STACK_SIZE 4096
#define SERIAL_TASK_PRIORITY 2
#define SERIAL_QUEUE_SIZE 10
#define MAX_MESSAGE_SIZE 128

// Estrutura para mensagens seriais
typedef struct {
    char message[MAX_MESSAGE_SIZE];
    uint8_t length;
} serial_message_t;

// Funções públicas do módulo
void serial_module_init(void);
bool serial_send_message(const char* message);
bool serial_send_message_from_isr(const char* message);

#endif // SERIAL_MODULE_H