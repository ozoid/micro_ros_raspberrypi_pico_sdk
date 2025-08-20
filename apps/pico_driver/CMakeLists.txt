cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
set(FREERTOS_KERNEL_PATH "/home/pi/micro_ros_raspberrypi_pico_sdk/FreeRTOS-Kernel/")
set(FREERTOS_PORT "GCC_ARM_CM0" CACHE STRING "Port for FreeRTOS")
set(FREERTOS_PORTABLE_DIR portable/GCC/ARM_CM0)
set(FREERTOS_CONFIG_FILE_DIRECTORY "/home/pi/micro_ros_raspberrypi_pico_sdk/apps/pico_driver/include")

project(pico_driver C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

include(FreeRTOS_Kernel_import.cmake)

add_executable(pico_driver
    combined_imu.cpp
    time_override.c
    ${CMAKE_SOURCE_DIR}/FreeRTOS-Kernel/portable/MemMang/heap_4.c
)

target_include_directories(pico_driver PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_SOURCE_DIR}/libmicroros/include
    ${CMAKE_SOURCE_DIR}/FreeRTOS-Kernel/include
    ${CMAKE_SOURCE_DIR}/FreeRTOS-Kernel/portable/GCC/ARM_CM0
)



target_link_libraries(pico_driver
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_i2c
    freertos_kernel
    microros
    m
)

target_compile_definitions(pico_driver PUBLIC
    RCUTILS_NO_POSIX_SUPPORT
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(pico_driver 1)
pico_enable_stdio_uart(pico_driver 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(pico_driver)
