# ESP-IDF only (controller and bridge). The STM32 build never sees this.
idf_component_register(
    SRCS "src/espnow_wifi.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_netif esp_event nvs_flash rc_protocol
)
