# Same source
file(GLOB_RECURSE SOURCE_FILES 
    ${CMAKE_SOURCE_DIR}/src/**/*.cpp
)

# Create test executable
file(GLOB_RECURSE TEST_FILES
    ./**/*.cpp
)

add_executable(unit_tests
    ${SOURCE_FILES}
    ${TEST_FILES}
)


# Set the output directory
set_target_properties(unit_tests
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# Include directories
target_include_directories(unit_tests
    PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/test
    ${GTEST_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(unit_tests
    PRIVATE
    GTest::gtest_main
    Eigen3::Eigen
)

# Add tests to CTest
add_test(NAME unit_tests 
         COMMAND ${CMAKE_BINARY_DIR}/bin/unit_tests)

# Optional: Add test labels for organization
set_tests_properties(unit_tests PROPERTIES LABELS "unit")