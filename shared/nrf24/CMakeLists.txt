# ESP-IDF component. The STM32 build adds src/nrf24.c and include/ directly.
idf_component_register(SRCS "src/nrf24.c" INCLUDE_DIRS "include")
