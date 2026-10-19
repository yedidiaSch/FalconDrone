idf_component_register(
    SRCS "main.c" "inputs.c" "nrf24_port_esp32.c"
    INCLUDE_DIRS "."
    REQUIRES rc_protocol nrf24 esp_driver_gpio esp_driver_spi esp_adc esp_rom
)
