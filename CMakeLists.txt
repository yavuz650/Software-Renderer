cmake_minimum_required(VERSION 3.10)

# set the project name
project(main)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/..")
set(CMAKE_VERBOSE_MAKEFILE on)

# add the executable
add_executable(main main.cpp model.cpp tgaimage.cpp)
target_compile_options(main PRIVATE -O3)

target_link_libraries(main SDL2)

