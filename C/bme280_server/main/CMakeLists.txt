# Read configuration from .env file at repo root
set(ENV_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../../.env")

if(EXISTS ${ENV_FILE})
    file(STRINGS ${ENV_FILE} ENV_CONTENTS)
    foreach(LINE ${ENV_CONTENTS})
        if(LINE MATCHES "^WIFI_SSID=(.*)$")
            set(WIFI_SSID "${CMAKE_MATCH_1}")
        elseif(LINE MATCHES "^WIFI_PASSWORD=(.*)$")
            set(WIFI_PASSWORD "${CMAKE_MATCH_1}")
        elseif(LINE MATCHES "^API_ENDPOINT=(.*)$")
            set(API_ENDPOINT "${CMAKE_MATCH_1}")
        elseif(LINE MATCHES "^DEVICE_ID=(.*)$")
            set(DEVICE_ID "${CMAKE_MATCH_1}")
        endif()
    endforeach()
else()
    message(FATAL_ERROR "Missing .env file at ${ENV_FILE}. Create it with WIFI_SSID, WIFI_PASSWORD, API_ENDPOINT, and DEVICE_ID.")
endif()

if(NOT DEFINED WIFI_SSID OR NOT DEFINED WIFI_PASSWORD)
    message(FATAL_ERROR ".env file must contain WIFI_SSID and WIFI_PASSWORD")
endif()

if(NOT DEFINED API_ENDPOINT)
    message(FATAL_ERROR ".env file must contain API_ENDPOINT")
endif()

if(NOT DEFINED DEVICE_ID)
    set(DEVICE_ID "esp32-bme280")
    message(STATUS "DEVICE_ID not set in .env, using default: ${DEVICE_ID}")
endif()

idf_component_register(
    SRCS "weather_server.c" "bme280_sensor.c" "wifi_manager.c" "led_status.c"
         "http_client.c" "data_poster.c" "sensor_reading.pb.c"
    INCLUDE_DIRS "."
)

# Pass configuration as compile definitions
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    WIFI_SSID="${WIFI_SSID}"
    WIFI_PASSWORD="${WIFI_PASSWORD}"
    API_ENDPOINT="${API_ENDPOINT}"
    DEVICE_ID="${DEVICE_ID}"
)

# Extra warnings for better error detection (only applies to main component)
target_compile_options(${COMPONENT_LIB} PRIVATE
#    -Wall                    # Standard warnings
#    -Wextra                  # Additional useful warnings
    -Wshadow                 # Warn when variable shadows another
#    -Wdouble-promotion       # Warn on implicit float->double
    -Wformat=2               # Strict printf/scanf format checking
    -Wconversion             # Warn on implicit type conversions
    -Wno-sign-conversion     # Suppress sign conversion noise
    -Wnull-dereference       # Warn on potential null pointer dereference
    -Wuninitialized          # Warn on uninitialized variable use
    -Wstrict-prototypes      # Require full function prototypes
    -Wmissing-prototypes     # Warn if function lacks prototype
)
