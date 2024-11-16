cmake_minimum_required(VERSION 3.8)
project(imu_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(
    ${PROJECT_NAME}_node
    src/IMUTest.cpp
    src/OPI_IMU.cpp
    src/ZEstimator.cpp
    src/Madgwick_Filter.cpp
    src/EMAFilter.cpp
    src/imu_test_node.cpp
)

ament_target_dependencies(
    ${PROJECT_NAME}_node
    rclcpp
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
    DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
