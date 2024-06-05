idf_component_register(SRCS "base/spi_bus.cpp" "spi_device.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES driver
                       PRIV_REQUIRES settings
                                     log
)