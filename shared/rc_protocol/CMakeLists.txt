# Header-only. ESP-IDF picks this up as a component; the STM32 build adds
# shared/rc_protocol/include to its include path directly.
idf_component_register(INCLUDE_DIRS "include")
