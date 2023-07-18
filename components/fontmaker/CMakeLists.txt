set(srcs "esp_font.c" "st7789_driver.c" "esp_font_ascii.c" )

idf_component_register(SRCS ${srcs} INCLUDE_DIRS "." REQUIRES driver tuyaos_config)
