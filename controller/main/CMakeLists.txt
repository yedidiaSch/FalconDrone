# Radio transport from menuconfig: FalconDrone → Radio transport. Only the
# sources switch: REQUIRES is resolved before sdkconfig is read, so both
# transports' components are listed and the linker drops the unused one.
if(CONFIG_FALCON_RADIO_NRF24)
    set(radio_srcs "radio_nrf24.c" "nrf24_port_esp32.c")
else()
    set(radio_srcs "radio_espnow.c")
endif()

idf_component_register(
    SRCS "main.c" "inputs.c" "gfx.c" "ui_draw.c" "ui.c" "oled.c" ${radio_srcs}
    INCLUDE_DIRS "."
    REQUIRES rc_protocol esp_driver_gpio esp_adc esp_rom esp_lcd esp_driver_i2c
             nrf24 esp_driver_spi espnow_wifi
)
