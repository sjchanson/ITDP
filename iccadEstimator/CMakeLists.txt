# CMake version number requirements
cmake_minimum_required(VERSION 3.1)

# Project information
project(iccadEstimator)

# Compile mode
set(CMAKE_BUILD_TYPE "Debug")

# Compile options
if(CMAKE_COMPILER_IS_GNUCXX)
    add_compile_options(-std=c++17 -Wall -O3 -fopenmp)
else()
    message(FATAL_ERROR "G++ NOT FOUND")
endif()
add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=YES)

# Close the warning
add_definitions(-w)

# Link legacy Libraries
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)

# Set
set(FLUTE_HOME ${PROJECT_SOURCE_DIR}/Flute)

# Flute Setting
add_definitions(-D_FLUTE_PATH)
add_definitions(-DPOWVFILE=\"${FLUTE_HOME}/POWV9.dat\")
add_definitions(-DPORTFILE=\"${FLUTE_HOME}/PORT9.dat\")

# Sub Directory
add_subdirectory(${FLUTE_HOME})
aux_source_directory(. DIR_SRCS)

# Link
link_directories(${PROJECT_SOURCE_DIR}/libs)

# Target
add_library(${PROJECT_NAME} STATIC
            ${DIR_SRCS})

target_link_libraries(${PROJECT_NAME} ui-timer)
target_link_libraries(${PROJECT_NAME} flute)
