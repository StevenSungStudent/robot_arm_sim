#include "RobotPosePublisher.hpp"
#include "iostream"

RobotPosePublisher::RobotPosePublisher() : 
    Node("pose_publisher")
{
    // broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    joint_states.name = {"base_link2turret", "turret2upperarm", "upperarm2forearm", "forearm2wrist", "wrist2hand", "gripper_left2hand", "gripper_right2hand"};
    joint_states.position = {0,0,0,0,0,0,0};

    timer = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&RobotPosePublisher::joint_publisher_callback, this));
}

RobotPosePublisher::~RobotPosePublisher()
{

}

void RobotPosePublisher::joint_publisher_callback(){
    joint_states.header.stamp = this->get_clock()->now();
    publisher->publish(joint_states);
}
