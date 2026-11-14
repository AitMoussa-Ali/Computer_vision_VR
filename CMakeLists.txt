cmake_minimum_required(VERSION 3.16)

project(vision_augmentee_q1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV 4 REQUIRED COMPONENTS
    core
    imgproc
    imgcodecs
    highgui
    videoio
    features2d
    calib3d
    video
)

add_executable(q1_tracking
    src/reference_tracker.cpp
    tests/q1/main.cpp
)

target_include_directories(q1_tracking PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(q1_tracking PRIVATE ${OpenCV_LIBS})
