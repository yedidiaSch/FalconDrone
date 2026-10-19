#ifndef NRF24_PORT_ESP32_H
#define NRF24_PORT_ESP32_H

/*
 * nRF24L01+ board glue for the ESP32 controller: SPI3_HOST (VSPI) at 4 MHz,
 * mode 0, hardware CS on PIN_NRF_CSN, CE on a plain GPIO. Pins in board.h.
 */

#include "esp_err.h"
#include "nrf24.h"

/* Set up the SPI bus, the device and the CE pin, and fill *port. */
esp_err_t nrf24_port_esp32_init(Nrf24Port *port);

#endif /* NRF24_PORT_ESP32_H */
