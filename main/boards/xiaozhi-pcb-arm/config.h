#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

// if CONFIG_xxx macros are needed, include sdkconfig.h first
#include <sdkconfig.h>

// ============================================================================
// Audio: INMP441 microphone (I2S RX) + MAX98357 amplifier (I2S TX)
// Both are simple I2S devices without a programmable codec, so the board uses
// the NoAudioCodecSimplex path (separate RX/TX channels).
// ============================================================================
#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

// INMP441 digital microphone
#define AUDIO_I2S_MIC_GPIO_SCK  GPIO_NUM_6
#define AUDIO_I2S_MIC_GPIO_WS   GPIO_NUM_5
#define AUDIO_I2S_MIC_GPIO_DIN  GPIO_NUM_4

// MAX98357 I2S amplifier (SD_MODE tied to 3.3V, always enabled, L/R output)
#define AUDIO_I2S_SPK_GPIO_DOUT GPIO_NUM_16
#define AUDIO_I2S_SPK_GPIO_BCLK GPIO_NUM_15
#define AUDIO_I2S_SPK_GPIO_LRCK GPIO_NUM_7

// ============================================================================
// Buttons / LED
// ============================================================================
#define BOOT_BUTTON_GPIO GPIO_NUM_0

// No onboard status LED is wired on this board; keep the alias for future use.
// #define BUILTIN_LED_GPIO GPIO_NUM_NC

// ============================================================================
// Main display: SSD1306-compatible 128x64 OLED on I2C (GPIO41 SDA / GPIO42 SCL)
// ============================================================================
#define DISPLAY_SDA_PIN GPIO_NUM_41
#define DISPLAY_SCL_PIN GPIO_NUM_42
#define DISPLAY_I2C_ADDRESS 0x3C
#define DISPLAY_WIDTH   128
#define DISPLAY_HEIGHT  64
#define DISPLAY_MIRROR_X true
#define DISPLAY_MIRROR_Y true

// ============================================================================
// Robot eyes: two 1.28" GC9A01 round LCDs sharing one SPI bus.
// CS is independent per panel (GPIO10 / GPIO9), all other lines are common.
// Both panels mirror the same framebuffer ("same content" requirement).
// ============================================================================
#define DISPLAY_GC9A01_SPI_HOST  SPI2_HOST
#define DISPLAY_GC9A01_SCK_PIN   GPIO_NUM_11
#define DISPLAY_GC9A01_MOSI_PIN  GPIO_NUM_12
#define DISPLAY_GC9A01_DC_PIN    GPIO_NUM_13
#define DISPLAY_GC9A01_RST_PIN   GPIO_NUM_14
#define DISPLAY_GC9A01_CS_EYE_L  GPIO_NUM_10
#define DISPLAY_GC9A01_CS_EYE_R  GPIO_NUM_9
#define DISPLAY_GC9A01_WIDTH     240
#define DISPLAY_GC9A01_HEIGHT    240

// ============================================================================
// Motors: dual AT8870 H-bridge drivers, 6-wire quadrature encoder motors.
// IN1/IN2 (PWM) control each channel; encoder A/B phases feed PCNT.
// Logic-side pins are schematic-confirmed: A-IN1=17, A-IN2=18, B-IN1=38,
// B-IN2=47.
// ============================================================================
#define MOTOR_A_IN1_PIN GPIO_NUM_17
#define MOTOR_A_IN2_PIN GPIO_NUM_18
#define MOTOR_B_IN1_PIN GPIO_NUM_38
#define MOTOR_B_IN2_PIN GPIO_NUM_47

// Encoder phase inputs (schematic-confirmed)
#define ENCODER_A_A_PIN GPIO_NUM_1
#define ENCODER_A_B_PIN GPIO_NUM_2
#define ENCODER_B_A_PIN GPIO_NUM_21
#define ENCODER_B_B_PIN GPIO_NUM_48

// Standby pins if wired to MCU; set to GPIO_NUM_NC when tied high on board.
#define MOTOR_STBY_PIN  GPIO_NUM_NC

// ============================================================================
// Lidar: UART RX for point-cloud stream + PWM for motor speed control
// ============================================================================
#define LIDAR_UART_NUM       UART_NUM_1
#define LIDAR_UART_RX_PIN    GPIO_NUM_39
#define LIDAR_UART_TX_PIN    GPIO_NUM_NC  // most lidars are RX-only
#define LIDAR_UART_BAUDRATE  230400
#define LIDAR_PWM_PIN        GPIO_NUM_40
#define LIDAR_PWM_FREQ_HZ    25000

// ============================================================================
// Network: this variant uses Wi-Fi only (WifiBoard). The ML307R footprint is
// populated on some units; those ship as a separate -4g variant.
// ============================================================================
// #define ML307_TX_PIN GPIO_NUM_20
// #define ML307_RX_PIN GPIO_NUM_19

#endif // _BOARD_CONFIG_H_
