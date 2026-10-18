#ifndef BOARD_H
#define BOARD_H

/*
 * ESP32-WROOM-32 DevKit pin map. Everything runs at 3.3 V — including the
 * joystick (its "+5V" pin goes to 3V3: more than 3.3 V on a GPIO damages it).
 *
 *   nRF24L01+   VCC→3V3 (+ 10-100 uF cap at the module), GND,
 *               CE→D4, CSN→D5, SCK→D18, MOSI→D23, MISO→D19
 *   OLED        SSD1306 128x64, VCC→3V3, GND, SDA→D21, SCL→D22
 *   Slider      ends→3V3/GND, wiper→D34
 *   Joystick    VRx→D32, VRy→D33, SW→D25
 *
 * Analog inputs must stay on ADC1 (GPIO32-39): ADC2 is unusable while the
 * Wi-Fi driver is active, so it is avoided altogether.
 */

#include "driver/gpio.h"

/* nRF24 — SPI3_HOST (VSPI) */
#define PIN_NRF_SCK     GPIO_NUM_18
#define PIN_NRF_MOSI    GPIO_NUM_23
#define PIN_NRF_MISO    GPIO_NUM_19
#define PIN_NRF_CSN     GPIO_NUM_5
#define PIN_NRF_CE      GPIO_NUM_4

/* SSD1306 OLED — I2C */
#define PIN_OLED_SDA    GPIO_NUM_21
#define PIN_OLED_SCL    GPIO_NUM_22
#define OLED_I2C_ADDR   0x3C

/* Analog inputs (ADC1) */
#define PIN_THROTTLE    GPIO_NUM_34   /* ADC1_CH6 */
#define PIN_STICK_X     GPIO_NUM_32   /* ADC1_CH4 — roll  */
#define PIN_STICK_Y     GPIO_NUM_33   /* ADC1_CH5 — pitch */

/* Joystick push button, active low (internal pull-up) */
#define PIN_STICK_SW    GPIO_NUM_25

#endif /* BOARD_H */
