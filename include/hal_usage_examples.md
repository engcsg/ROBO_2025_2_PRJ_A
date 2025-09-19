/**
 * @file hal_usage_examples.md
 * @brief Exemplos de uso do Hardware Abstraction Layer (HAL)
 * 
 * Este arquivo contém exemplos práticos de como usar o HAL
 * para adicionar novos componentes de hardware ao projeto.
 */

// =============================================================================
// EXEMPLO 1: Adicionando um LED adicional
// =============================================================================

```cpp
#include "hardware_hal.h"

void setup_additional_led() {
    // Usar LED definido no HAL
    pinMode(HAL_LED_USER_1_PIN, OUTPUT);
    digitalWrite(HAL_LED_USER_1_PIN, LOW);
}

void blink_user_led() {
    digitalWrite(HAL_LED_USER_1_PIN, HIGH);
    delay(500);
    digitalWrite(HAL_LED_USER_1_PIN, LOW);
    delay(500);
}
```

// =============================================================================
// EXEMPLO 2: Configurando um botão com pull-up
// =============================================================================

```cpp
#include "hardware_hal.h"

void setup_button() {
    pinMode(HAL_BUTTON_USER_1_PIN, INPUT_PULLUP);
}

bool is_button_pressed() {
    return digitalRead(HAL_BUTTON_USER_1_PIN) == LOW;
}
```

// =============================================================================
// EXEMPLO 3: Configurando PWM para controle de motor
// =============================================================================

```cpp
#include "hardware_hal.h"

void setup_motor_pwm() {
    // Configurar canal PWM
    ledcSetup(HAL_PWM_CHANNEL_0, HAL_PWM_FREQUENCY, HAL_PWM_RESOLUTION);
    ledcAttachPin(HAL_PWM_MOTOR_1_PIN, HAL_PWM_CHANNEL_0);
}

void set_motor_speed(uint8_t speed) {
    // speed: 0-255
    ledcWrite(HAL_PWM_CHANNEL_0, speed);
}
```

// =============================================================================
// EXEMPLO 4: Lendo sensor analógico
// =============================================================================

```cpp
#include "hardware_hal.h"

void setup_analog_sensor() {
    analogReadResolution(HAL_ADC_RESOLUTION);
}

uint16_t read_sensor_voltage_mv() {
    uint16_t adc_value = analogRead(HAL_ADC_SENSOR_1_PIN);
    return HAL_ADC_TO_MV(adc_value);
}
```

// =============================================================================
// EXEMPLO 5: Configurando I2C para display OLED
// =============================================================================

```cpp
#include "hardware_hal.h"
#include <Wire.h>

void setup_i2c_display() {
    Wire.begin(HAL_I2C_SDA_PIN, HAL_I2C_SCL_PIN);
    Wire.setClock(HAL_I2C_FREQUENCY);
    
    // Verificar se display está conectado
    Wire.beginTransmission(HAL_OLED_I2C_ADDRESS);
    if (Wire.endTransmission() == 0) {
        Serial.println("OLED display found!");
    }
}
```

// =============================================================================
// EXEMPLO 6: Adicionando novo módulo de sensor
// =============================================================================

```cpp
// sensor_module.h
#ifndef SENSOR_MODULE_H
#define SENSOR_MODULE_H

#include "hardware_hal.h"

// Usar definições do HAL
#define TEMP_SENSOR_PIN HAL_TEMP_SENSOR_PIN
#define LIGHT_SENSOR_PIN HAL_LDR_SENSOR_PIN

void sensor_module_init(void);
float read_temperature(void);
uint16_t read_light_level(void);

#endif
```

```cpp
// sensor_module.cpp
#include "sensor_module.h"

void sensor_module_init(void) {
    pinMode(HAL_TEMP_SENSOR_PIN, INPUT);
    analogReadResolution(HAL_ADC_RESOLUTION);
}

float read_temperature(void) {
    // Implementação específica do sensor
    return 25.0; // Exemplo
}

uint16_t read_light_level(void) {
    return analogRead(HAL_LDR_SENSOR_PIN);
}
```

// =============================================================================
// EXEMPLO 7: Validação de pinos antes do uso
// =============================================================================

```cpp
#include "hardware_hal.h"

bool setup_custom_gpio(uint8_t pin) {
    if (!HAL_IS_VALID_GPIO(pin)) {
        Serial.printf("Error: Pin %d is not a valid GPIO\n", pin);
        return false;
    }
    
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    return true;
}
```

// =============================================================================
// VANTAGENS DO HAL
// =============================================================================

1. **Centralização**: Todos os pinos em um local
2. **Documentação**: Cada pino tem propósito definido
3. **Reutilização**: Fácil mudança de pinos
4. **Validação**: Macros para verificar validade
5. **Portabilidade**: Facilita migração para outras placas
6. **Organização**: Estrutura clara do hardware
7. **Manutenção**: Mudanças centralizadas
8. **Debugging**: Fácil identificação de conflitos

// =============================================================================
// BOAS PRÁTICAS
// =============================================================================

1. Sempre incluir "hardware_hal.h" nos módulos
2. Usar as definições HAL_* em vez de números diretos
3. Documentar novos pinos adicionados
4. Verificar conflitos antes de definir novos pinos
5. Usar estruturas de configuração para periféricos complexos
6. Manter compatibilidade com versões anteriores quando possível