idf_component_register(SRCS "communication.cpp" "ftp_client.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES driver
                                esp_wifi
                       PRIV_REQUIRES log
                                     settings
                                     lwip
)