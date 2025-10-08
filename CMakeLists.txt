cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_micro_ros_example C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(FREERTOS_KERNEL_PATH "/home/pi/micro_ros_raspberrypi_pico_sdk/FreeRTOS-Kernel/")
set(FREERTOS_PORT "GCC_RP2040" CACHE STRING "Port for FreeRTOS")
set(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_SOURCE_DIR}/apps/pico_driver/include")
set(FREERTOS_HEAP "4" CACHE STRING "")

add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE
    "${FREERTOS_CONFIG_FILE_DIRECTORY}"
)
target_compile_definitions(freertos_config INTERFACE
    projCOVERAGE_TEST=0
    RCUTILS_NO_POSIX_SUPPORT
    UXR_CONFIG_CUSTOM_TRANSPORT_MTU=1024
    RMW_UXRCE_MAX_TRANSPORT_MTU=1024
    RMW_UXRCE_STREAM_HISTORY_OUTPUT=8
    RMW_UXRCE_STREAM_HISTORY_INPUT=8
    RMW_UXRCE_MAX_MESSAGE_SIZE=2048
)

set(CMAKE_THREAD_LIBS_INIT "" CACHE STRING "" FORCE)
set(THREADS_PREFER_PTHREAD_FLAG OFF CACHE BOOL "" FORCE)
set(UCLIENT_PROFILE_MULTITHREAD ON CACHE BOOL "Enable multithreading in uclient" FORCE)

pico_sdk_init()

include(micro_ros.cmake)
add_subdirectory(FreeRTOS-Kernel)
add_subdirectory(apps)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")
