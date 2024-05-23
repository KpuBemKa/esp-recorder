idf_component_register(SRCS "i2s_sampler.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES driver
                       PRIV_REQUIRES log
)