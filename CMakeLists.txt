cmake_minimum_required(VERSION 3.15)
project(calib_k4a LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(k4a REQUIRED)
find_package(k4arecord REQUIRED)
find_package(Open3D REQUIRED)
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)

################################################################################
# Source

set(INCLUDE_FILES
    include/camera_extrinsics.hpp
    include/calib_k4a.hpp
)

set(SOURCE_FILES
    ${INCLUDE_FILES}
    src/camera_extrinsics.cpp
    src/calib_k4a.cpp
)

message(STATUS "Open3D : ${Open3D_LIBRARIES}")
include_directories(${OpenCV_INCLUDE_DIRS})


################################################################################
# Targets
add_executable(calib_k4a ${SOURCE_FILES})
target_link_libraries(calib_k4a PUBLIC
    k4a # Kinect SDK
    k4arecord 
    opencv_core opencv_imgproc opencv_calib3d opencv_aruco
    ${Open3D_LIBRARIES} # Implicitly includes Eigen
    apriltag::apriltag
    stdc++fs # filesystem
)
target_include_directories(calib_k4a PUBLIC include)
