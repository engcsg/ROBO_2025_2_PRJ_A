#ifndef BLINK_MODULE_H
#define BLINK_MODULE_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Configurações do módulo de blink
#define LED_PIN 2
#define BLINK_TASK_STACK_SIZE 2048
#define BLINK_TASK_PRIORITY 1

// Estrutura para configuração do blink
typedef struct {
    uint32_t on_time_ms;
    uint32_t off_time_ms;
    bool enabled;
} blink_config_t;

// Funções públicas do módulo
void blink_module_init(void);
void blink_set_config(blink_config_t config);
void blink_enable(bool enable);

#endif // BLINK_MODULE_H