idf_component_register(
    SRCS "main.cpp" "icm20948.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_spi esp_driver_i2c esp_wifi esp_netif esp_event nvs_flash esp_timer lwip
)
