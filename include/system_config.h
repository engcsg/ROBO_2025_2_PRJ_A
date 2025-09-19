#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

// Configurações gerais do sistema
#define SYSTEM_BAUD_RATE 115200

// Prioridades das tasks (quanto maior o número, maior a prioridade)
#define PRIORITY_SERIAL 2
#define PRIORITY_BLINK 1

// Tamanhos de stack para cada task
#define STACK_SIZE_SERIAL 4096
#define STACK_SIZE_BLINK 2048

// Timeouts e delays
#define SERIAL_TIMEOUT_MS 100
#define DEFAULT_BLINK_DELAY_MS 500

#endif // SYSTEM_CONFIG_H