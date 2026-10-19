idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES rc_protocol espnow_wifi esp_driver_uart esp_driver_gpio
)
