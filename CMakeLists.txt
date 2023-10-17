cmake_minimum_required(VERSION 3.12)

include(pico-sdk/pico_sdk_init.cmake)

project(pico_motors C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

set(PICO_BOARD pico_w)

pico_sdk_init()

link_directories(micro_ros_raspberrypi_pico_sdk/libmicroros)

if (PICO_CYW43_SUPPORTED)
    if (NOT TARGET pico_cyw43_arch)
        message("Skipping Pico W as support is not available")
    else()

        if (DEFINED ENV{WIFI_SSID} AND (NOT WIFI_SSID))
            set(WIFI_SSID $ENV{WIFI_SSID})
            message("Using WIFI_SSID from environment ('${WIFI_SSID}')")
        endif()

        if (DEFINED ENV{WIFI_PASSWORD} AND (NOT WIFI_PASSWORD))
            set(WIFI_PASSWORD $ENV{WIFI_PASSWORD})
            message("Using WIFI_PASSWORD from environment")
        endif()

        set(WIFI_SSID "${WIFI_SSID}" CACHE INTERNAL "WiFi SSID for examples")
        set(WIFI_PASSWORD "${WIFI_PASSWORD}" CACHE INTERNAL "WiFi password for examples")
    endif()
endif()

add_subdirectory(ADS1x15_PICO)

# Motor Driver
add_library(motor_driver
        src/motor/motor_driver.cpp)
target_include_directories(motor_driver
        PUBLIC
        include/motor)
target_link_libraries(motor_driver pico_stdlib hardware_pwm)

# ADCWrapper
add_library(adc_wrapper
        src/adc_wrapper/adc_wrapper.cpp)
target_include_directories(adc_wrapper
        PUBLIC
        include/adc_wrapper)
target_link_libraries(adc_wrapper pico_stdlib pico_ads1x15)

# Sensor
add_library(sensor
        src/sensor/sensor.cpp)
target_include_directories(sensor
        PUBLIC
        include/sensor)
target_link_libraries(sensor pico_stdlib adc_wrapper)

# SensorManager
add_library(sensor_manager
        src/sensor_manager/sensor_manager.cpp)
target_include_directories(sensor_manager
        PUBLIC
        include/sensor_manager)
target_link_libraries(sensor_manager pico_stdlib sensor adc_wrapper)

# ROSWrapper
add_library(ros_wrapper
        src/microros/ros_wrapper.cpp
        src/microros/pico_udp_transport.cpp)
target_include_directories(ros_wrapper
        PUBLIC
        include/microros
        micro_ros_raspberrypi_pico_sdk/libmicroros/include
        ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(ros_wrapper pico_stdlib pico_cyw43_arch_lwip_threadsafe_background microros)

# run
add_executable(run
        src/run.cpp)
target_link_libraries(run pico_stdlib ros_wrapper motor_driver sensor sensor_manager)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

pico_enable_stdio_usb(run 1)
pico_enable_stdio_uart(run 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

pico_add_extra_outputs(run)


add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        )
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()

# Hardware-specific examples in subdirectories:

