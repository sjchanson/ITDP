# CMake version number requirements
cmake_minimum_required(VERSION 3.1)

# Compile options
if(CMAKE_COMPILER_IS_GNUCXX)
    add_compile_options(-std=c++17 -Wall -O0 -fopenmp)
else()
    message(FATAL_ERROR "G++ NOT FOUND")
endif()
add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=YES)

# include the head file.
include_directories(${PROJECT_SOURCE_DIR}/include)
message(${PROJECT_SOURCE_DIR}++++)

# Link legacy Libraries
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)

add_library(sequentialCluster SHARED
            ClockTree.cpp
            SequentialBase.cpp
            SequentialOperator.cpp
            SkewConstraintGraph.cpp)
