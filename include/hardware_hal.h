#ifndef HARDWARE_HAL_H
#define HARDWARE_HAL_H

/**
 * @file hardware_hal.h
 * @brief Hardware Abstraction Layer (HAL) para ESP32
 * 
 * Este arquivo centraliza todas as definições de hardware do projeto,
 * incluindo pinos, configurações de periféricos e constantes relacionadas
 * ao hardware específico do ESP32.
 * 
 * Todos os módulos devem incluir este header em vez de definir
 * pinos e configurações de hardware diretamente.
 */

#include <Arduino.h>

// =============================================================================
// DEFINIÇÕES GERAIS DO MICROCONTROLADOR
// =============================================================================

#define MCU_FREQUENCY_MHZ       240
#define MCU_VOLTAGE_3V3         3300    // mV
#define MCU_ADC_RESOLUTION      12      // bits

// =============================================================================
// CONFIGURAÇÕES DE PINOS DIGITAIS
// =============================================================================

// --- LEDs ---
#define HAL_LED_BUILTIN_PIN     2       // LED interno do ESP32
#define HAL_LED_STATUS_PIN      HAL_LED_BUILTIN_PIN
#define HAL_LED_USER_1_PIN      4       // LED adicional 1
#define HAL_LED_USER_2_PIN      5       // LED adicional 2

// --- BOTÕES ---
#define HAL_BUTTON_BOOT_PIN     0       // Botão BOOT do ESP32
#define HAL_BUTTON_USER_1_PIN   14      // Botão de usuário 1
#define HAL_BUTTON_USER_2_PIN   27      // Botão de usuário 2

// --- RELÉS ---
#define HAL_RELAY_1_PIN         16      // Relé 1
#define HAL_RELAY_2_PIN         17      // Relé 2
#define HAL_RELAY_3_PIN         18      // Relé 3
#define HAL_RELAY_4_PIN         19      // Relé 4

// --- GPIO DE PROPÓSITO GERAL ---
#define HAL_GPIO_1_PIN          12      // GPIO 1
#define HAL_GPIO_2_PIN          13      // GPIO 2
#define HAL_GPIO_3_PIN          25      // GPIO 3
#define HAL_GPIO_4_PIN          26      // GPIO 4

// =============================================================================
// CONFIGURAÇÕES DE COMUNICAÇÃO SERIAL
// =============================================================================

// --- UART0 (Serial principal) ---
#define HAL_UART0_TX_PIN        1       // TX padrão
#define HAL_UART0_RX_PIN        3       // RX padrão
#define HAL_UART0_BAUD_RATE     115200

// --- UART1 (Serial adicional) ---
#define HAL_UART1_TX_PIN        17      // TX secundário
#define HAL_UART1_RX_PIN        16      // RX secundário
#define HAL_UART1_BAUD_RATE     9600

// --- UART2 (Serial adicional) ---
#define HAL_UART2_TX_PIN        4       // TX terciário
#define HAL_UART2_RX_PIN        5       // RX terciário
#define HAL_UART2_BAUD_RATE     9600

// =============================================================================
// CONFIGURAÇÕES I2C
// =============================================================================

#define HAL_I2C_SDA_PIN         21      // SDA padrão
#define HAL_I2C_SCL_PIN         22      // SCL padrão
#define HAL_I2C_FREQUENCY       100000  // 100kHz

// =============================================================================
// CONFIGURAÇÕES SPI
// =============================================================================

#define HAL_SPI_MOSI_PIN        23      // MOSI padrão
#define HAL_SPI_MISO_PIN        19      // MISO padrão
#define HAL_SPI_SCK_PIN         18      // SCK padrão
#define HAL_SPI_SS_PIN          5       // SS padrão

// =============================================================================
// CONFIGURAÇÕES PWM
// =============================================================================

#define HAL_PWM_RESOLUTION      8       // 8 bits (0-255)
#define HAL_PWM_FREQUENCY       5000    // 5kHz

// --- Canais PWM ---
#define HAL_PWM_CHANNEL_0       0
#define HAL_PWM_CHANNEL_1       1
#define HAL_PWM_CHANNEL_2       2
#define HAL_PWM_CHANNEL_3       3

// --- Pinos PWM ---
#define HAL_PWM_LED_PIN         HAL_LED_BUILTIN_PIN
#define HAL_PWM_SERVO_1_PIN     32      // Servo 1
#define HAL_PWM_SERVO_2_PIN     33      // Servo 2
#define HAL_PWM_MOTOR_1_PIN     25      // Motor 1
#define HAL_PWM_MOTOR_2_PIN     26      // Motor 2

// =============================================================================
// CONFIGURAÇÕES ADC (Conversor Analógico-Digital)
// =============================================================================

#define HAL_ADC_RESOLUTION      12      // 12 bits (0-4095)
#define HAL_ADC_VREF_MV         3300    // Tensão de referência em mV

// --- Pinos ADC ---
#define HAL_ADC_SENSOR_1_PIN    36      // ADC1_CH0 - Sensor 1
#define HAL_ADC_SENSOR_2_PIN    39      // ADC1_CH3 - Sensor 2
#define HAL_ADC_POTENTIOMETER_PIN 34    // ADC1_CH6 - Potenciômetro
#define HAL_ADC_BATTERY_PIN     35      // ADC1_CH7 - Monitoramento bateria

// =============================================================================
// CONFIGURAÇÕES DAC (Conversor Digital-Analógico)
// =============================================================================

#define HAL_DAC_RESOLUTION      8       // 8 bits (0-255)

// --- Pinos DAC ---
#define HAL_DAC_OUTPUT_1_PIN    25      // DAC1
#define HAL_DAC_OUTPUT_2_PIN    26      // DAC2

// =============================================================================
// CONFIGURAÇÕES DE SENSORES
// =============================================================================

// --- Sensor de Temperatura (DS18B20) ---
#define HAL_TEMP_SENSOR_PIN     4

// --- Sensor Ultrassônico (HC-SR04) ---
#define HAL_ULTRASONIC_TRIGGER_PIN  5
#define HAL_ULTRASONIC_ECHO_PIN     18

// --- Sensor PIR (Movimento) ---
#define HAL_PIR_SENSOR_PIN      15

// --- Sensor de Luz (LDR) ---
#define HAL_LDR_SENSOR_PIN      HAL_ADC_SENSOR_1_PIN

// =============================================================================
// CONFIGURAÇÕES DE ATUADORES
// =============================================================================

// --- Buzzer ---
#define HAL_BUZZER_PIN          23

// --- Motor DC com Driver ---
#define HAL_MOTOR_A_PIN1        16      // Motor A - Pino 1
#define HAL_MOTOR_A_PIN2        17      // Motor A - Pino 2
#define HAL_MOTOR_B_PIN1        18      // Motor B - Pino 1
#define HAL_MOTOR_B_PIN2        19      // Motor B - Pino 2

// --- Servo Motores ---
#define HAL_SERVO_1_PIN         HAL_PWM_SERVO_1_PIN
#define HAL_SERVO_2_PIN         HAL_PWM_SERVO_2_PIN

// =============================================================================
// CONFIGURAÇÕES DE DISPLAY
// =============================================================================

// --- Display LCD I2C ---
#define HAL_LCD_I2C_ADDRESS     0x27    // Endereço I2C comum para LCD
#define HAL_LCD_COLUMNS         16      // Colunas do display
#define HAL_LCD_ROWS            2       // Linhas do display

// --- Display OLED I2C ---
#define HAL_OLED_I2C_ADDRESS    0x3C    // Endereço I2C comum para OLED
#define HAL_OLED_WIDTH          128     // Largura em pixels
#define HAL_OLED_HEIGHT         64      // Altura em pixels

// =============================================================================
// CONFIGURAÇÕES DE REDE
// =============================================================================

// --- WiFi ---
#define HAL_WIFI_DEFAULT_SSID   "ESP32_Robot"
#define HAL_WIFI_DEFAULT_PASS   "12345678"

// --- Bluetooth ---
#define HAL_BT_DEVICE_NAME      "ESP32_Robot_BT"

// =============================================================================
// CONFIGURAÇÕES DE SISTEMA
// =============================================================================

// --- Watchdog Timer ---
#define HAL_WDT_TIMEOUT_MS      8000    // 8 segundos

// --- Timers ---
#define HAL_TIMER_0             0
#define HAL_TIMER_1             1
#define HAL_TIMER_2             2
#define HAL_TIMER_3             3

// =============================================================================
// MACROS AUXILIARES
// =============================================================================

// Macro para conversão ADC para mV
#define HAL_ADC_TO_MV(adc_value) ((adc_value * HAL_ADC_VREF_MV) / ((1 << HAL_ADC_RESOLUTION) - 1))

// Macro para conversão mV para DAC
#define HAL_MV_TO_DAC(mv_value) ((mv_value * ((1 << HAL_DAC_RESOLUTION) - 1)) / HAL_MCU_VOLTAGE_3V3)

// Macro para validação de pino GPIO
#define HAL_IS_VALID_GPIO(pin) ((pin >= 0) && (pin <= 39) && (pin != 6) && (pin != 7) && \
                                (pin != 8) && (pin != 9) && (pin != 10) && (pin != 11))

// =============================================================================
// ESTRUTURAS DE CONFIGURAÇÃO
// =============================================================================

// Estrutura para configuração de pinos
typedef struct {
    uint8_t pin;
    uint8_t mode;       // INPUT, OUTPUT, INPUT_PULLUP, etc.
    bool    initial_state;
} hal_pin_config_t;

// Estrutura para configuração PWM
typedef struct {
    uint8_t pin;
    uint8_t channel;
    uint32_t frequency;
    uint8_t resolution;
} hal_pwm_config_t;

// Estrutura para configuração I2C
typedef struct {
    uint8_t sda_pin;
    uint8_t scl_pin;
    uint32_t frequency;
} hal_i2c_config_t;

// Estrutura para configuração SPI
typedef struct {
    uint8_t mosi_pin;
    uint8_t miso_pin;
    uint8_t sck_pin;
    uint8_t ss_pin;
    uint32_t frequency;
} hal_spi_config_t;

// =============================================================================
// FUNÇÕES DE INICIALIZAÇÃO DO HAL
// =============================================================================

// Inicialização geral do HAL
void hal_init(void);

// Inicialização específica de periféricos
void hal_gpio_init(void);
void hal_uart_init(void);
void hal_i2c_init(void);
void hal_spi_init(void);
void hal_pwm_init(void);
void hal_adc_init(void);

// =============================================================================
// COMPATIBILIDADE COM VERSÕES ANTERIORES
// =============================================================================

// Para manter compatibilidade com código existente
#define LED_PIN                 HAL_LED_BUILTIN_PIN
#define BUTTON_PIN              HAL_BUTTON_BOOT_PIN

#endif // HARDWARE_HAL_H