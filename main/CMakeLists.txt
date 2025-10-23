
idf_component_register(SRCS main.c
                       PRIV_REQUIRES esp_https_server driver nvs_flash esp_wifi esp_event freertos json dns_server esp_app_format led_strip esp_http_client esp_https_ota mbedtls
                       )
