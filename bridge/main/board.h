#ifndef BRIDGE_BOARD_H
#define BRIDGE_BOARD_H

/*
 * Bridge ⇄ STM32 UART pins. Cross the lines and share ground:
 *
 *   bridge TX → STM32 PA10 (USART1_RX, Nucleo D2)
 *   bridge RX ← STM32 PA9  (USART1_TX, Nucleo D8)
 *   GND       — GND
 *   5V / VIN  ← 5 V (the BEC). Not the Nucleo's 3V3: Wi-Fi peaks ~300 mA.
 *
 * Both sides are 3.3 V logic, so the wires connect directly.
 */

#include "driver/gpio.h"
#include "driver/uart.h"

#define BRIDGE_UART      UART_NUM_1

#if CONFIG_IDF_TARGET_ESP32
/* ESP32 DevKit: the pins marked TX2 / RX2. */
#define PIN_BRIDGE_TX    GPIO_NUM_17
#define PIN_BRIDGE_RX    GPIO_NUM_16
#elif CONFIG_IDF_TARGET_ESP32C3
/* ESP32-C3 (DevKitM / Super Mini): free pins, away from the strapping pins
 * 2/8/9 and the console on 20/21. */
#define PIN_BRIDGE_TX    GPIO_NUM_7
#define PIN_BRIDGE_RX    GPIO_NUM_6
#else
#error "Pick UART pins for this target in board.h"
#endif

#endif /* BRIDGE_BOARD_H */
