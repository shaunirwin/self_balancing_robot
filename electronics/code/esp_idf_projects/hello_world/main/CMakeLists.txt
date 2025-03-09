idf_component_register(SRCS "hello_worldd.cpp"
                    REQUIRES esp_driver_uart esp_driver_gpio
                    INCLUDE_DIRS "include")
