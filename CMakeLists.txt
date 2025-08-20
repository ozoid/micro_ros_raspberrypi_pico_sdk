cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_micro_ros_example C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(FREERTOS_KERNEL_PATH "/home/pi/micro_ros_raspberrypi_pico_sdk/FreeRTOS-Kernel/")
set(FREERTOS_PORT "GCC_ARM_CM0" CACHE STRING "Port for FreeRTOS")
set(FREERTOS_PORTABLE_DIR portable/GCC/ARM_CM0)
set(FREERTOS_CONFIG_FILE_DIRECTORY "/home/pi/micro_ros_raspberrypi_pico_sdk/apps/pico_driver/include")
set(CMAKE_THREAD_LIBS_INIT "" CACHE STRING "" FORCE)
set(THREADS_PREFER_PTHREAD_FLAG OFF CACHE BOOL "" FORCE)

pico_sdk_init()

#include(FreeRTOS_Kernel_import.cmake)
include(micro_ros.cmake)
add_subdirectory(FreeRTOS-Kernel)
add_subdirectory(apps)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")
