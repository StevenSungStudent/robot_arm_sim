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


class RobotPosePublisher : public rclcpp::Node
{
public:
    RobotPosePublisher();
    ~RobotPosePublisher();

private:
    const unsigned short update_frequency;
    std::vector<double> delta_angle;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    sensor_msgs::msg::JointState joint_states;
    sensor_msgs::msg::JointState current_joint_states;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
    void joint_publisher_callback();
    void command_callback(const std_msgs::msg::String & command);
};

#endif