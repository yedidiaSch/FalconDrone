#ifndef __NRF24_PORT_STM32_H__
#define __NRF24_PORT_STM32_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf24.h"

/*
 * nRF24L01+ board glue for the Nucleo-F446RE (wiring on CN10, all 3.3 V):
 *
 *   nRF24   Nucleo
 *   VCC     3V3 (+ 10-100 uF cap at the module)
 *   GND     GND
 *   CE      PB1   (CN10-24)
 *   CSN     PB12  (CN10-16)
 *   SCK     PB13  (CN10-30)   SPI2, mode 0, 5.6 MHz
 *   MISO    PB14  (CN10-28)
 *   MOSI    PB15  (CN10-26)
 *   IRQ     not connected (the driver polls)
 *
 * SPI2 and the two GPIOs are set up by CubeMX (spi.c, gpio.c).
 */

/* The port for nrf24_init(). Also enables the DWT cycle counter used for
 * microsecond delays. */
const Nrf24Port *Nrf24Port_Stm32(void);

#ifdef __cplusplus
}
#endif

#endif /* __NRF24_PORT_STM32_H__ */
