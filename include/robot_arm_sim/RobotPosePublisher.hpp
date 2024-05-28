#ifndef ROBOT_POSE_PUBLISHER_HPP
#define ROBOT_POSE_PUBLISHER_HPP

#pragma once

//Info: This code is for moving the robot sim.

#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_msgs/msg/tf_message.h"
#include "tf2/exceptions.h"


class RobotPosePublisher : public rclcpp::Node
{
public:
    RobotPosePublisher();
    ~RobotPosePublisher();

private:
    const unsigned short max_pwm_;
    const unsigned short min_pwm_;
    const unsigned short update_frequency_;
    // bool cup_in_gripper;
    // geometry_msgs::msg::Transform cup_position;
    std::vector<double> delta_angle;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    sensor_msgs::msg::JointState joint_states;
    sensor_msgs::msg::JointState current_joint_states;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;

    // std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    // std::unique_ptr<tf2_ros::Buffer> tf_buffer_;


    void joint_publisher_callback();
    void command_callback(const std_msgs::msg::String& command);
    // void tf_callback(const tf2_msgs::msg::TFMessage& tf_message);
    // void parse_transform_data();
    double PWM_to_angle(long value);
    double PWM_to_meter(long value);
};

#endif