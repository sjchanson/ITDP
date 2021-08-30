# CMake version number requirements
cmake_minimum_required(VERSION 3.1)

# Project information
project(iTDP)

# Compile mode
SET(CMAKE_BUILD_TYPE "Debug")

SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++17 -no-pie -O0 -Wall -g2 -ggdb -D_GLIBCXX_USE_CXX11_ABI=0 -fopenmp")

# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++0x -no-pie -O0 -Wall -g2 -ggdb -D_GLIBCXX_USE_CXX11_ABI=0 -fopenmp -pg")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++0x -no-pie -O0 -Wall -g2 -ggdb -D_GLIBCXX_USE_CXX11_ABI=0 -fopenmp -fsanitize=address")

# # Build Library Or Binary
# option(ENABLE_GOOGLE_TEST "RUN GOOGLE TEST" ON)

# # Finding Headers.
# add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=ON)

# # OpenMP
# FIND_PACKAGE( OpenMP REQUIRED)
# if(OPENMP_FOUND)
# message("OPENMP FOUND")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
# endif()

# # Set
# SET(FLUTE_HOME ${PROJECT_SOURCE_DIR}/Flute)
SET(ADAPTER_HOME ${PROJECT_SOURCE_DIR}/dbAdapter)
SET(CLUSTER_HOME ${PROJECT_SOURCE_DIR}/sequentialCluster)
# SET(REVISE_HOME ${PROJECT_SOURCE_DIR}/reviseFile)
SET(GOOGLETEST_HOME ${PROJECT_SOURCE_DIR}/test)

# # Flute File.
# add_definitions(-D_FLUTE_PATH)
# add_definitions(-DPOWVFILE=\"${FLUTE_HOME}/POWV9.dat\")
# add_definitions(-DPORTFILE=\"${FLUTE_HOME}/PORT9.dat\")

# # Header file directory
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/src/iCTS/include)

# # include_directories(${PROJECT_SOURCE_DIR}/include/common)


# # Directory
# add_subdirectory(${FLUTE_HOME})
add_subdirectory(${ADAPTER_HOME})
add_subdirectory(${CLUSTER_HOME})
add_subdirectory(${PROJECT_SOURCE_DIR}/DME)

# add_subdirectory(${REVISE_HOME})
# aux_source_directory(${PROJECT_SOURCE_DIR}/src DIR_SRCS)

# # Link
link_directories(${PROJECT_SOURCE_DIR}/iccadEstimator/libs)

# # Build library
# if(ENABLE_GOOGLE_TEST) 
#     add_library(${PROJECT_NAME} ${DIR_SRCS})
#     message("BUILD LIBRARY ONLY")
#     # Unit testing setup
    # add_subdirectory(${GOOGLETEST_HOME})
#     # start test
    # enable_testing()
# else()
#     add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/main.cpp ${DIR_SRCS})
#         message("BUILD BINARY")
# endif()
#     # Target
#     target_link_libraries(${PROJECT_NAME} ui-timer)
#     target_link_libraries(${PROJECT_NAME} flute)
#     target_link_libraries(${PROJECT_NAME} sequentalCluster)
#     target_link_libraries(${PROJECT_NAME} reviseFile)

add_subdirectory(${PROJECT_SOURCE_DIR}/iccadEstimator)
add_library(${PROJECT_NAME}  
            ./src/Logger.cpp
            ./src/Parameter.cpp
)
target_link_libraries(${PORJECT_NAME} iccadEstimator)
target_link_libraries(${PROJECT_NAME} Adapter)
target_link_libraries(${PORJECT_NAME} sequentialCluster)
target_link_libraries(${PROJECT_NAME} DME)





