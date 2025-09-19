#include <Arduino.h>
#include "blink_module.h"
#include "serial_module.h"
#include "system_config.h"

void setup() {
  // Inicializar comunicação serial primeiro
  Serial.begin(SYSTEM_BAUD_RATE);
  
  // Aguardar um pouco para estabilizar
  delay(100);
  
  // Inicializar módulos
  serial_module_init();  // Inicializar primeiro para ter sistema de log
  blink_module_init();   // Inicializar módulo de blink
  
  // Enviar mensagem de inicialização do sistema
  delay(200); // Aguardar módulos inicializarem
  serial_send_message("System initialization complete!");
  
  // Configurar blink inicial
  blink_config_t initial_config = {500, 500, true};
  blink_set_config(initial_config);
}

void loop() {
  // O loop principal fica vazio pois as tasks do FreeRTOS gerenciam tudo
  // Pequeno delay para evitar watchdog reset
  delay(1000);
}