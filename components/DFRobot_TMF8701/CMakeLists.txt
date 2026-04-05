idf_component_register(
    SRCS
        "DFRobot_TMF8701_espidf.cpp"
        "tmf8701_c_api.cpp"
    INCLUDE_DIRS
        "."
        "drv"
    REQUIRES
        driver           # because headers include driver/i2c.h + driver/gpio.h
    PRIV_REQUIRES
        esp_timer        # because .cpp includes esp_timer.h
)
