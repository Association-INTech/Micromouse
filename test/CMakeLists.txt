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
    ../src/hl/graph.cpp
    ../src/hl/maze.cpp
    ../src/hl/curve.cpp
    ../src/hl/trajectory.cpp
)
target_include_directories(ratatech_hl PUBLIC ../include)
target_compile_options(ratatech_hl PRIVATE -Wall -Wextra)

foreach(name IN ITEMS test_maze test_graph test_curve test_trajectory)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} PRIVATE ratatech_hl GTest::gtest_main)
    gtest_discover_tests(${name})
endforeach()
