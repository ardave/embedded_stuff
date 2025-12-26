# Read WiFi credentials from .env file at repo root
set(ENV_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../../.env")

if(EXISTS ${ENV_FILE})
    file(STRINGS ${ENV_FILE} ENV_CONTENTS)
    foreach(LINE ${ENV_CONTENTS})
        if(LINE MATCHES "^WIFI_SSID=(.*)$")
            set(WIFI_SSID "${CMAKE_MATCH_1}")
        elseif(LINE MATCHES "^WIFI_PASSWORD=(.*)$")
            set(WIFI_PASSWORD "${CMAKE_MATCH_1}")
        endif()
    endforeach()
else()
    message(FATAL_ERROR "Missing .env file at ${ENV_FILE}. Create it with WIFI_SSID and WIFI_PASSWORD.")
endif()

if(NOT DEFINED WIFI_SSID OR NOT DEFINED WIFI_PASSWORD)
    message(FATAL_ERROR ".env file must contain WIFI_SSID and WIFI_PASSWORD")
endif()

idf_component_register(
    SRCS "bme280_server.c" "wifi_manager.c" "led_status.c" "http_server.c"
    INCLUDE_DIRS "."
)

# Pass WiFi credentials as compile definitions
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    WIFI_SSID="${WIFI_SSID}"
    WIFI_PASSWORD="${WIFI_PASSWORD}"
)
