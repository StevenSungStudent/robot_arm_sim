#ifndef CUP_POSE_PUBLISHER_HPP
#define CUP_POSE_PUBLISHER_HPP

#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_msgs/msg/tf_message.h"
#include "tf2/exceptions.h"


class CupPosePublisher : public rclcpp::Node
{
public:
    CupPosePublisher();
    ~CupPosePublisher();

private:
    void pose_publisher_callback();
    void parse_transform_data();
    void update_cup_position();

    double distance_2d(double x1, double y1, double x2, double y2);

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
    rclcpp::TimerBase::SharedPtr timer;
    
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    geometry_msgs::msg::Transform hand_position;
    geometry_msgs::msg::Transform gripper_left_position;
    geometry_msgs::msg::Transform gripper_right_position;

    geometry_msgs::msg::TransformStamped current_pose;
    unsigned short update_frequency;
};

#endif // CUP_POSE_PUBLISHER_HPP
