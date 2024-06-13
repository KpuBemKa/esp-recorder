idf_component_register(SRCS "ftp_client.cpp" "communication.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES driver
                                esp_wifi
                       PRIV_REQUIRES log
                                     settings
)