cmake_minimum_required(VERSION 3.10)
project(Outset VERSION 0.1)

include(CTest)
enable_testing()

# GoogleTest requires at least C++17
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)

FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
#   GIT_TAG        3.4.0 # or latest commit/tag
)

FetchContent_MakeAvailable(eigen)

# Add the executable target
file(GLOB_RECURSE SOURCE_FILES
    src/*.cpp
    src/**/*.cpp
)
# list(APPEND SOURCE_FILES src/main.cpp)
add_executable(Outset ${SOURCE_FILES})
message(STATUS "Source files: ${SOURCE_FILES}")

# Include directories for Outset
target_include_directories(Outset PRIVATE 
    ${CMAKE_SOURCE_DIR}/src
)



# Link libraries for Earthbound
target_link_libraries(Outset PRIVATE
    # GTest::gtest
    Eigen3::Eigen
)

# Add test directory
add_subdirectory(test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG") #  -march=native does NOT work on M1 mac