idf_component_register(SRCS "main.c" "../src/mqtt.c" "../src/wifi_sta.c"
                    INCLUDE_DIRS "." "../inc"
                    REQUIRES nvs_flash esp_wifi esp_event esp_netif esp_driver_uart esp_driver_gpio)