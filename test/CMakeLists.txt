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

add_executable(test_hl
    test_hl.cpp
    ../src/hl/graph.cpp
    ../src/hl/maze.cpp
)

target_include_directories(test_hl PRIVATE ../include)
target_compile_options(test_hl PRIVATE -Wall -Wextra)
target_link_libraries(test_hl GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(test_hl)
