message("iTDP Test")

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/include)

add_definitions(-DCMAKE_BUILD_TYPE=Debug)
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=YES)

aux_source_directory(./src SOURCE_FILE)

add_executable(iTDPTest ${SOURCE_FILE})

link_directories(${CMAKE_SOURCE_DIR}/iccadEstimator/libs)
message("CURRENT DIRATORY : ${CMAKE_SOURCE_DIR}")

target_link_libraries(
    iTDPTest
    PUBLIC
    iTDP
    iccadEstimator
    sequentialUnitCluster
    # flute
    # ui-timer
    ${GTEST_BOTH_LIBRARIES}
    pthread
)

# ADD TO TEST
# gtest_discover_tests(iTDPTest)