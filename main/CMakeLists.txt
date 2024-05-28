idf_component_register(SRCS "main.cpp"
                            "config.cpp"
                            "server.cpp"    
                INCLUDE_DIRS "."
                PRIV_REQUIRES wav_file
                                sd_card
                                i2s_sampler
                                esp_netif
                                esp_event
                                esp_wifi
                                nvs_flash
                                esp_timer
                                settings
                                communication
)
