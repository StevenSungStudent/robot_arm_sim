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

// #include "joint_state.hpp"


class RobotPosePublisher : public rclcpp::Node
{
public:
    RobotPosePublisher();
    ~RobotPosePublisher();

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;

    sensor_msgs::msg::JointState joint_states;
    void joint_publisher_callback();
};

#endif