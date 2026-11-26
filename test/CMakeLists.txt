# test/ has its own CMakeLists.txt to avoid the arm toolchain
cmake_minimum_required(VERSION 3.22)

project(ratatech_test CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
FetchContent_Declare(
    googletest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG v1.18.0
)
FetchContent_MakeAvailable(googletest)

enable_testing()

include(GoogleTest)

add_library(ratatech_hl STATIC
    ../src/hl/maze.cpp
    ../src/hl/curve_rank1.cpp
)
target_include_directories(ratatech_hl PUBLIC ../include)

if(MSVC)
    target_compile_options(ratatech_hl PRIVATE /W4 /permissive-)
else()
    target_compile_options(ratatech_hl PRIVATE
                           -Wall -Wextra -Wpedantic)
endif()

foreach(name IN ITEMS test_maze test_graph test_curve test_trajectory)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} PRIVATE ratatech_hl GTest::gtest_main)
    gtest_discover_tests(${name})
    if(MSVC)
        target_compile_options(${name} PRIVATE /W4 /permissive-)
    else()
        target_compile_options(${name} PRIVATE
                               -Wall -Wextra -Wpedantic)
    endif()
endforeach()
