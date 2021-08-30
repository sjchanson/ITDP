message("iTDP Test")

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

add_definitions(-DCMAKE_BUILD_TYPE=Debug)
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=YES)

aux_source_directory(./src SOURCE_FILE)

add_executable(iTDPTest ${SOURCE_FILE})

link_directories(${PROJECT_SOURCE_DIR}/iccadEstimator/libs)
message("CURRENT DIRATORY : ${PROJECT_SOURCE_DIR}")

# Add head file.
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/DME)
include_directories(${PROJECT_SOURCE_DIR}/dbAdapter)
include_directories(${PROJECT_SOURCE_DIR}/iccadEstimator)
include_directories(${PROJECT_SOURCE_DIR}/sequentialCluster)

target_link_libraries(
    iTDPTest
    PUBLIC
    iTDP
    iccadEstimator
    sequentialCluster
    # flute
    # ui-timer
    ${GTEST_BOTH_LIBRARIES}
    pthread
)

# ADD TO TEST
# gtest_discover_tests(iTDPTest)