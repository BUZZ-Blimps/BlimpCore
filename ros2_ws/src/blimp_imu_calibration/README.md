# Blimp IMU Calibration

## Dev Sources
- Calibration Concept: https://chionophilous.wordpress.com/2011/10/24/accelerometer-calibration-iv-1-implementing-gauss-newton-on-an-atmega/
- Willie Prior Calibration Code: https://github.com/williamdwarke/TeensyFC
    - libraries/TeensyMPU9255/src/util/AccCalibration.cpp
    - libraries/TeensyMPU9255/src/TeensyMPU9255.cpp
        - Lines 488 - 543
- BlimpCore/ros2_ws/src/imu_test/src/OPI_IMU.cpp
- BlimpCore/ros2_ws/src/imu_test/include/OPI_IMU.hpp

## Dev Notes
This will be shipped in a ROS2 Humble package, but this will not be a node.
It will just be a normal C++ executable. Hopefully this works.

We could have just put this as a separate C++ project elsewhere, completely independent of ROS, but this way, we can still store and run it within the ROS framework. :)