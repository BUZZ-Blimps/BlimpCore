cmake_minimum_required(VERSION 3.8)
project(catching_blimp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(
    SOURCE_FILES
    src/BangBang.cpp
    src/OPI_IMU.cpp
    src/Brushless.cpp
    src/EMAFilter.cpp
    # src/Gimbal.cpp
    src/ZEstimator.cpp
    src/Madgwick_Filter.cpp
    # src/MotorControl.cpp
    src/optical_ekf.cpp
    src/PID.cpp
    src/Servo.cpp
    src/tripleBallGrabber.cpp
    src/math_helpers.cpp
    src/CatchingBlimp.cpp
    src/CatchingBlimpStateMachine.cpp
    src/catching_blimp_node.cpp
    src/MotorControl_V2.cpp
    src/TOF_Sense.cpp
)

add_executable(
    ${PROJECT_NAME}_node
    ${SOURCE_FILES}
)

ament_target_dependencies(
    ${PROJECT_NAME}_node 
    rclcpp 
    std_msgs 
    geometry_msgs 
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    Eigen3
)

target_include_directories(
    ${PROJECT_NAME}_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_node Eigen3::Eigen wiringPi)

install(
    TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)

install(
    DIRECTORY calibration
    DESTINATION share/${PROJECT_NAME}/
)

install(
    DIRECTORY param
    DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS scripts/battery_monitor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  CODE "execute_process(COMMAND chmod +x ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/battery_monitor_node.py)"
)

ament_package()
