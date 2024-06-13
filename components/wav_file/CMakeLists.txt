idf_component_register(SRCS "wav_reader.cpp"
                            "wav_writer.cpp"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES log
                                     settings
)