
# my_project/CMakeLists.txt
cmake_minimum_required(VERSION 3.12 FATAL_ERROR)
project(exe)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(CXX_FLAGS -g  -Wall -Wno-unused-function -Wno-maybe-uninitialized -Wno-unknown-pragmas -fmax-errors=1)
add_compile_options(${CXX_FLAGS})
add_compile_definitions(_GLIBCXX_ASSERTIONS)


add_subdirectory(${CMAKE_SOURCE_DIR}/.. ${CMAKE_SOURCE_DIR}/build-lib)

add_executable(${PROJECT_NAME} test.cpp)


target_include_directories(${PROJECT_NAME} PRIVATE ../include )
target_link_libraries(${PROJECT_NAME} PRIVATE sim-tools)
