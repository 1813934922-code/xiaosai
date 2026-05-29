cmake_minimum_required(VERSION 3.22)

# -----------------------------
# 基础配置
# -----------------------------
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME xiaosai)
project(${CMAKE_PROJECT_NAME})
message("Build type: ${CMAKE_BUILD_TYPE}")

enable_language(C ASM)

# -----------------------------
# 创建可执行文件
# -----------------------------
add_executable(${CMAKE_PROJECT_NAME}
        User/Device/buzzer.c
        User/It/uart_it.c
        User/It/uart_it.h
        User/Motor/lq_step.c
        User/Sensor/gy901.c
        User/Sensor/abslute_angle_sensor.c
        User/Tool/eeprom.c
        User/It/timer_it.h
        User/Tool/pendulum.c
        User/Tool/pendulum.h)

# -----------------------------
# STM32CubeMX 生成代码
# -----------------------------
add_subdirectory(cmake/stm32cubemx)

# -----------------------------
# 添加 User 文件夹所有子目录到 include path
# -----------------------------
file(GLOB_RECURSE USER_INCLUDE_DIRS LIST_DIRECTORIES true "User/*")
foreach(dir ${USER_INCLUDE_DIRS})
    if(IS_DIRECTORY "${dir}")
        target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE "${dir}")
    endif()
endforeach()

# -----------------------------
# 添加 User 下所有 C 文件
# -----------------------------
file(GLOB_RECURSE USER_SOURCES "User/*.c")
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${USER_SOURCES})

# -----------------------------
# 链接 STM32CubeMX 库
# -----------------------------
target_link_libraries(${CMAKE_PROJECT_NAME} stm32cubemx)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)