#ifndef STEPPER_TASK_H
#define STEPPER_TASK_H

#include <Arduino.h>

// Configurações do driver de passo - ajuste conforme seu hardware
#define STEPPER_PIN_STEP GPIO_NUM_13
#define STEPPER_PIN_DIR GPIO_NUM_12
#define STEPPER_PIN_ENABLE GPIO_NUM_14

// Se o pino ENABLE é ativo em nível baixo (LOW), deixe 1; se ativo em HIGH, defina 0
#define STEPPER_ENABLE_ACTIVE_LOW 1

// Pulsos e temporizações (ajuste velocidade aqui)
#define STEPPER_PULSE_WIDTH_US 10      // duração do pulso (microsegundos)
#define STEPPER_STEP_DELAY_US 1000    // intervalo entre pulsos (microsegundos)

// Número de passos a mover em cada sentido
#define STEPPER_STEPS_PER_MOVE 200

// Inicializa e inicia a task do stepper; chame no setup()
void stepper_task_init(void);

#endif // STEPPER_TASK_H
