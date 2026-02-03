/**
 * @file catching_blimp_node.cpp
 * @brief Entry point for the Catching Blimp ROS2 node.
 *
 * Initializes ROS2, creates and spins the CatchingBlimp node (which handles
 * sensors, state machine, and motor control), then shuts down on exit.
 */

#include "CatchingBlimp.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CatchingBlimp>());
    rclcpp::shutdown();
    return 0;
}
