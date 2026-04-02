idf_component_register(SRCS "take_picture.c" "app_wifi.c"
                            "firebase_client.c"
                            "schedule_runner.c"
                       PRIV_REQUIRES esp_wifi nvs_flash wifi_provisioning bt protocomm
                                     esp_http_client json esp_timer
                       INCLUDE_DIRS ".")