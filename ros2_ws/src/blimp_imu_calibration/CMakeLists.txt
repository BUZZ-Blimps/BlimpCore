cmake_minimum_required(VERSION 3.8)
project(blimp_imu_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(
    ${PROJECT_NAME}_node
    src/main.cpp
    src/AccelerometerCalibrator.cpp
    src/OPI_IMU.cpp
)

ament_target_dependencies(
    ${PROJECT_NAME}_node
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

ament_package()