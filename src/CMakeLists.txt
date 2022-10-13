set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/..")
set(CMAKE_VERBOSE_MAKEFILE on)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# add the executable
add_executable(main main.cpp tgaimage.cpp rasterizer.cpp shader.cpp 
               triangle.cpp utils.cpp graphicsPipeline.cpp)

target_link_libraries(main SDL2)