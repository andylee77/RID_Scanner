# ESP-IDF CMakeLists.txt for RID Scanner

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash esp_wifi esp_netif esp_event esp_timer driver
)
