#include "stepper_task.h"
#include "serial_module.h"
#include "system_config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Parâmetros da task
#define STEPPER_TASK_STACK_SIZE 4096
#define STEPPER_TASK_PRIORITY 1

static void stepper_task(void* pvParameters) {
  (void) pvParameters;

  // Configurar pinos
  pinMode(STEPPER_PIN_STEP, OUTPUT);
  pinMode(STEPPER_PIN_DIR, OUTPUT);
  pinMode(STEPPER_PIN_ENABLE, OUTPUT);

  // Níveis de enable
  const uint8_t enable_active_level = (STEPPER_ENABLE_ACTIVE_LOW ? LOW : HIGH);
  const uint8_t enable_inactive_level = (enable_active_level == LOW) ? HIGH : LOW;

  // Garantir estado inicial: desabilitado
  digitalWrite(STEPPER_PIN_ENABLE, enable_inactive_level);
  digitalWrite(STEPPER_PIN_STEP, LOW);

  // Mensagem inicial
  serial_send_message("Stepper task started");

  for (;;) {
    // Mover para um lado
    digitalWrite(STEPPER_PIN_DIR, LOW);
    digitalWrite(STEPPER_PIN_ENABLE, enable_active_level); // habilita driver
    serial_send_message("Stepper: movendo 1000 passos (DIR=LOW)");

    for (int i = 0; i < STEPPER_STEPS_PER_MOVE; ++i) {
      digitalWrite(STEPPER_PIN_STEP, HIGH);
      delayMicroseconds(STEPPER_PULSE_WIDTH_US);
      digitalWrite(STEPPER_PIN_STEP, LOW);
      // tempo restante até o próximo pulso
      delayMicroseconds(STEPPER_STEP_DELAY_US - STEPPER_PULSE_WIDTH_US);
    }

    digitalWrite(STEPPER_PIN_ENABLE, enable_inactive_level); // desabilita driver
    vTaskDelay(pdMS_TO_TICKS(200)); // pequena pausa

    // Mover para o outro lado
    digitalWrite(STEPPER_PIN_DIR, HIGH);
    digitalWrite(STEPPER_PIN_ENABLE, enable_active_level); // habilita driver
    serial_send_message("Stepper: movendo 1000 passos (DIR=HIGH)");

    for (int i = 0; i < STEPPER_STEPS_PER_MOVE; ++i) {
      digitalWrite(STEPPER_PIN_STEP, HIGH);
      delayMicroseconds(STEPPER_PULSE_WIDTH_US);
      digitalWrite(STEPPER_PIN_STEP, LOW);
      delayMicroseconds(STEPPER_STEP_DELAY_US - STEPPER_PULSE_WIDTH_US);
    }

    digitalWrite(STEPPER_PIN_ENABLE, enable_inactive_level); // desabilita driver

    // Pausa entre ciclos
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void stepper_task_init(void) {
  // Criar a task do FreeRTOS
  BaseType_t res = xTaskCreate(
    stepper_task,
    "stepper",
    STEPPER_TASK_STACK_SIZE,
    NULL,
    STEPPER_TASK_PRIORITY,
    NULL
  );

  if (res != pdPASS) {
    serial_send_message("Failed to create stepper task");
  }
}
