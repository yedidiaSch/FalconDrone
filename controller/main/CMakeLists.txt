idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES rc_protocol nrf24 esp_driver_gpio
)
