cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

add_executable(pico_driver
    combined_imu.cpp
    ${CMAKE_SOURCE_DIR}/pico_uart_transport.c
    ${CMAKE_SOURCE_DIR}/FreeRTOS-Kernel/portable/MemMang/heap_4.c
)

target_include_directories(pico_driver PUBLIC
    ${CMAKE_SOURCE_DIR}/apps/pico_driver/include
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_SOURCE_DIR}/
    ${CMAKE_SOURCE_DIR}/libmicroros/include

)

add_compile_definitions(
        PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE=1
        TUD_OPT_RP2040_USB_DEVICE_ENUMERATION_FIX=1
    )

target_link_libraries(pico_driver
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_i2c
    freertos_kernel
    #FreeRTOS-Kernel-Heap4
    freertos_config
    microros
    tinyusb_additions
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
