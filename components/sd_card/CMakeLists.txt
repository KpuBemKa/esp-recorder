idf_component_register(SRCS "sd_card.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES driver
                       PRIV_REQUIRES log
                                     fatfs
)