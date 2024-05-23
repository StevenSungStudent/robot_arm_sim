#include "rclcpp/rclcpp.hpp"
#include "CupPosePublisher.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CupPosePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
