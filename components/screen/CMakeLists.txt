idf_component_register(SRCS "screen_driver.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES u8g2
                                spi_driver
                       PRIV_REQUIRES settings
                                     log
)