#include <CatchingBlimp.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<blimp>());
    firstMessageTime = micros()/MICROS_TO_SEC;
    rclcpp::shutdown();
    return 0;
}