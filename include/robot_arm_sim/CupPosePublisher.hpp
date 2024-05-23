#ifndef CUP_POSE_PUBLISHER_HPP
#define CUP_POSE_PUBLISHER_HPP

#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

class CupPosePublisher : public rclcpp::Node
{
public:
    CupPosePublisher();
    ~CupPosePublisher();

private:
    void pose_publisher_callback();
    void command_callback(const std_msgs::msg::String & command);

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
    rclcpp::TimerBase::SharedPtr timer;

    geometry_msgs::msg::PoseStamped current_pose;
    unsigned short update_frequency;
};

#endif // CUP_POSE_PUBLISHER_HPP
