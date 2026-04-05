idf_component_register(SRCS "audio_player.c" "speaker.c" "tof.c" "dispenser.c" "ultrasonic.c" "main.c" "app_wifi.c"
                            "firebase_client.c"
                            "schedule_runner.c"
                       PRIV_REQUIRES esp_wifi nvs_flash wifi_provisioning bt protocomm
                                     esp_http_client json esp_timer driver DFRobot_TMF8701 
                       INCLUDE_DIRS ".")