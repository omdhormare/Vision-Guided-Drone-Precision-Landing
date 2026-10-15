
cmake_minimum_required(VERSION 3.18)

project(VisionGuidedLanding LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(drone
    src/main.cpp
    src/landing.cpp
    src/vision.cpp
)

target_include_directories(drone PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
    /ucrt64/include/opencv5
)

target_link_libraries(drone PRIVATE ${OpenCV_LIBS})

if(MSVC)
    target_compile_options(drone PRIVATE /W4)
else()
    target_compile_options(drone PRIVATE -Wall -Wextra -Wpedantic)
endif()

enable_testing()

add_executable(test_pid
    tests/test_pid.cpp
    src/landing.cpp
)

target_include_directories(test_pid PRIVATE include)

add_test(NAME pid_test COMMAND test_pid)
