#ifndef HARDWARE_HAL_H
#define HARDWARE_HAL_H

#include <Arduino.h>

/**
 * @file hardware_hal.h
 * @brief Hardware Abstraction Layer para o projeto ESP32.
 *
 * Este arquivo centraliza todos os pinos e parâmetros dos periféricos
 * utilizados pelas tasks. Qualquer alteração física deve ser feita aqui.
 */

// =============================================================================
// LEDS
// =============================================================================
#define HAL_LED_BUILTIN_PIN        GPIO_NUM_2

// =============================================================================
// SERIAL (UART0 nativo - usado pelo monitor serial)
// =============================================================================
#define HAL_SERIAL_PORT            Serial
#define HAL_SERIAL_BAUD_RATE       9600
#define HAL_SERIAL_CONFIG          SERIAL_8N1
#define HAL_SERIAL_TX_PIN          GPIO_NUM_1
#define HAL_SERIAL_RX_PIN          GPIO_NUM_3

// =============================================================================
// DRIVERS TB6600 - MOTOR BASE
// =============================================================================
#define HAL_STEPPER_BASE_STEP_PIN          GPIO_NUM_16  // RXD2
#define HAL_STEPPER_BASE_DIR_PIN           GPIO_NUM_17  // TXD2
#define HAL_STEPPER_BASE_ENABLE_PIN        GPIO_NUM_18  // D18
#define HAL_STEPPER_BASE_ENABLE_ACTIVE_LOW true

// =============================================================================
// DRIVERS TB6600 - MOTOR BRAÇO
// =============================================================================
#define HAL_STEPPER_ARM_STEP_PIN           GPIO_NUM_27  // D27
#define HAL_STEPPER_ARM_DIR_PIN            GPIO_NUM_26  // D26
#define HAL_STEPPER_ARM_ENABLE_PIN         GPIO_NUM_25  // D25
#define HAL_STEPPER_ARM_ENABLE_ACTIVE_LOW  true

// =============================================================================
// POTENCIÔMETROS (LEITURA ADC)
// =============================================================================
#define HAL_POT_BASE_PIN                   GPIO_NUM_34  // ADC1_6   - D34
#define HAL_POT_ARM_PIN                    GPIO_NUM_35  // ADC1_7   - D35
#define HAL_POT_ADC_MIN_VALUE              0
#define HAL_POT_ADC_MAX_VALUE              4095

// =============================================================================
// PARÂMETROS COMPARTILHADOS DE STEPPER
// =============================================================================
#define HAL_STEPPER_DEFAULT_PULSE_WIDTH_US 10
#define HAL_STEPPER_DEFAULT_STEP_DELAY_US  500

#endif  // HARDWARE_HAL_H
