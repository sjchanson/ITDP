# CMake version number requirements
cmake_minimum_required(VERSION 3.1)

# Project information
project(iTDP)

# Compile mode
SET(CMAKE_BUILD_TYPE "Debug")

SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++0x -no-pie -O0 -Wall -g2 -ggdb -D_GLIBCXX_USE_CXX11_ABI=0 -fopenmp")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++0x -no-pie -O0 -Wall -g2 -ggdb -D_GLIBCXX_USE_CXX11_ABI=0 -fopenmp -pg")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++0x -no-pie -O0 -Wall -g2 -ggdb -D_GLIBCXX_USE_CXX11_ABI=0 -fopenmp -fsanitize=address")

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


# Set
SET(FLUTE_HOME ${PROJECT_SOURCE_DIR}/Flute)
SET(CLUSTER_HOME ${PROJECT_SOURCE_DIR}/sequentialCluster)

# Header file directory
include_directories(${PROJECT_SOURCE_DIR}/library)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR})


# Directory

add_subdirectory(${FLUTE_HOME})
add_subdirectory(${CLUSTER_HOME})
aux_source_directory(. DIR_SRCS)

# Link
link_directories(${PROJECT_SOURCE_DIR}/libs)

# Target
add_executable(iTDPlacer ${DIR_SRCS})
target_link_libraries(iTDPlacer ui-timer)
target_link_libraries(iTDPlacer flute)
target_link_libraries(iTDPlacer sequentalCluster)
