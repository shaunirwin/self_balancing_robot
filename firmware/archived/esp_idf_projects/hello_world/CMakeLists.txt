# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# set(CMAKE_CXX_FLAGS "{$CMAKE_CXX_FLAGS} -std=c++20")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(hello_worldd)
