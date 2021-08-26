# CMake version number requirements
cmake_minimum_required(VERSION 3.1)

# Compile options
if(CMAKE_COMPILER_IS_GNUCXX)
    add_compile_options(-std=c++17 -Wall -O0 -fopenmp)
else()
    message(FATAL_ERROR "G++ NOT FOUND")
endif()
add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=YES)

# Add head file.
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/iccadEstimator)

# Link legacy Libraries
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)

add_library(Adapter SHARED
            Iccad2015Adapter.cpp)