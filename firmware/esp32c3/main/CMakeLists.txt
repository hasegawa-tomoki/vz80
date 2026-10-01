idf_component_register(SRCS "main.c" "wifi.c" "link.c" "web.c" "discovery.c" "sd.c" "cache.c" "files.c" "sdfw.c" "led.c"
                       INCLUDE_DIRS "."
                       REQUIRES nvs_flash esp_wifi esp_netif esp_event esp_http_server driver esp_timer app_update fatfs sdmmc)
# The Web UI is embedded gzip-compressed (served with Content-Encoding: gzip): a third of the size.
set(WEB_SRC ${CMAKE_CURRENT_LIST_DIR}/../web/index.html)
set(WEB_GZ ${CMAKE_BINARY_DIR}/index.html.gz)
add_custom_command(OUTPUT ${WEB_GZ}
    COMMAND ${PYTHON} -c "import gzip,sys; open(sys.argv[2],'wb').write(gzip.compress(open(sys.argv[1],'rb').read(), 9))" ${WEB_SRC} ${WEB_GZ}
    DEPENDS ${WEB_SRC} VERBATIM)
target_add_binary_data(${COMPONENT_LIB} ${WEB_GZ} BINARY DEPENDS ${WEB_GZ})
