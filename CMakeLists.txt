# CMake version number requirements
cmake_minimum_required(VERSION 3.1)

# Project information
project(iTDP)

# Compile mode
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++0x -no-pie -O0 -Wall -g2 -ggdb -D_GLIBCXX_USE_CXX11_ABI=0 -fopenmp")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++0x -no-pie -O0 -Wall -g2 -ggdb -D_GLIBCXX_USE_CXX11_ABI=0 -fopenmp -fsanitize=address")


# Set
SET(FLUTE_HOME ${PROJECT_SOURCE_DIR}/Flute)
SET(OPERATOR_HOME ${PROJECT_SOURCE_DIR}/operator)

# Header file directory
include_directories(${PROJECT_SOURCE_DIR}/library)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR})


# Directory
add_subdirectory(${FLUTE_HOME})
add_subdirectory(${OPERATOR_HOME})
aux_source_directory(. DIR_SRCS)

# Link
link_directories(${PROJECT_SOURCE_DIR}/libs)

# Target
add_executable(iTDPlacer ${DIR_SRCS})
target_link_libraries(iTDPlacer ui-timer)
target_link_libraries(iTDPlacer flute)
target_link_libraries(iTDPlacer operator)
