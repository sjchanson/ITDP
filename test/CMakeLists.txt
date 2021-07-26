message("iTDP Test")

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/include)

add_definitions(-DCMAKE_BUILD_TYPE=Debug)
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)

aux_source_directory(./src SOURCE_FILE)

add_executable(iTDPTest ${SOURCE_FILE})

target_link_libraries(
    iTDPTest
    PUBLIC
    iTDP
    ${GTEST_BOTH_LIBRARIES}
    pthread
)

# ADD TO TEST
# gtest_discover_tests(iTDPTest)