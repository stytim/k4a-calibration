# Extrinsic Calibration for Multiple Azure Kinect Cameras

This calibration tool only requires 1 frame of recording mkv files from each Azure Kinect camera. It first uses color frame for [AprilTag](https://github.com/AprilRobotics/apriltag) marker or ChArUco board detection and initial camera pose estimation. Then depth image from each camera is transformed to color camera to generate colored point cloud, which is used for [Colored ICP algorithm](https://openaccess.thecvf.com/content_ICCV_2017/papers/Park_Colored_Point_Cloud_ICCV_2017_paper.pdf). 

![Calibration](/img/image.jpg?raw=true "Left: Marker detection without ICP. Right: After ICP.")


The final result is saved as JSON file to be used in other software and PLY file for verifying calibration conveniently in any point cloud viewing software such as MeshLab.

## Prerequisites

* Azure Kinect SDK
* OpenCV
* AprilTag
* Open3D

## Building
Tested on Ubuntu 18.04 and 20.04
```
mkdir build && cd build
cmake ..
make
```

## Running
```
./calib_k4a <master.mkv> <sub1.mkv>...
```

## Important Note
The final extrinsic is transformed into depth camera coordinate system and also into OpenGL convention.

## Todo
* Test with [Teaser++](https://github.com/MIT-SPARK/TEASER-plusplus) for initial pose estimation
