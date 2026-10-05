cmake_minimum_required(VERSION 3.22)

set(CMAKE_TOOLCHAIN_FILE
    ${CMAKE_CURRENT_SOURCE_DIR}/cubemx/cmake/gcc-arm-none-eabi.cmake
)

project(ratatech C CXX ASM)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_subdirectory(cubemx)

target_sources(${CMAKE_PROJECT_NAME}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/app_main.cpp
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/mouse.cpp

    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/ll/ir_array.cpp
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/ll/motor_pid.cpp

    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/hl/maze.cpp
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/hl/graph.cpp
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/hl/trajectory.cpp
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/hl/curve.cpp
)

add_custom_target(flash
    openocd
        -f interface/stlink.cfg
        -f target/stm32g4x.cfg
        -c "program cubemx/ratatech.elf verify reset exit"
)
