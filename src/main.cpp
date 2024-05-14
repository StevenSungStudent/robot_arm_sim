#include "rclcpp/rclcpp.hpp"
#include "RobotPosePublisher.hpp"

int main(int argc, char const *argv[])
{
    /* code */
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotPosePublisher>());
    rclcpp::shutdown();
    return 0;
}
