idf_component_register(
    SRCS "main.c" "inputs.c" "nrf24_port_esp32.c" "gfx.c" "ui_draw.c" "ui.c" "oled.c"
    INCLUDE_DIRS "."
    REQUIRES rc_protocol nrf24 esp_driver_gpio esp_driver_spi esp_adc esp_rom esp_lcd esp_driver_i2c
)
