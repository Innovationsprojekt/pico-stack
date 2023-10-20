cmake_minimum_required(VERSION 3.12)

include(pico-sdk/external/pico_sdk_import.cmake)

project(pico_motors C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

set(PICO_BOARD pico_w CACHE STRING "Board type")

pico_sdk_init()

link_directories(micro_ros_raspberrypi_pico_sdk/libmicroros)

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

# MotorManager
add_library(motor_manager
        src/motor_manager/motor_manager.cpp)
target_include_directories(motor_manager
        PUBLIC
        include/motor_manager)
target_link_libraries(motor_manager pico_stdlib motor_driver)

# ROSWrapper
add_library(ros_wrapper
        src/microros/ros_wrapper.c
        src/microros/pico_udp_transport.c)
target_link_libraries(ros_wrapper pico_stdlib hardware_pwm microros)
target_include_directories(ros_wrapper
        PUBLIC
        include/microros
        micro_ros_raspberrypi_pico_sdk/libmicroros/include)
target_link_libraries(ros_wrapper pico_cyw43_arch_lwip_threadsafe_background)

# run
add_executable(run
        src/run.cpp)
target_link_libraries(run pico_stdlib ros_wrapper motor_driver motor_manager sensor sensor_manager)

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

