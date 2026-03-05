idf_component_register(SRCS "take_picture.c" "app_wifi.c"
                       PRIV_REQUIRES esp_wifi nvs_flash wifi_provisioning bt protocomm
                       INCLUDE_DIRS ".")