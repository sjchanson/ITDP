# CMake version number requirements
cmake_minimum_required(VERSION 3.1)

# Compile options
if(CMAKE_COMPILER_IS_GNUCXX)
    add_compile_options(-std=c++17 -Wall -O0 -fopenmp)
else()
    message(FATAL_ERROR "G++ NOT FOUND")
endif()
add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=YES)

# Link legacy Libraries
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)

add_library(DME SHARED
            CtsBase.cpp
            DME.cpp)   