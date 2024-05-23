#include "CupPosePublisher.hpp"

CupPosePublisher::CupPosePublisher() : Node("cup_pose_publisher"), update_frequency(10)
{
    publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("cup_pose", 10);
    subscription = this->create_subscription<std_msgs::msg::String>("cup_command", 10, std::bind(&CupPosePublisher::command_callback, this, std::placeholders::_1));

    current_pose.header.frame_id = "world";
    current_pose.pose.position.x = 0.3;
    current_pose.pose.position.y = 0.0;
    current_pose.pose.position.z = 0.05;
    current_pose.pose.orientation.x = 0.0;
    current_pose.pose.orientation.y = 0.0;
    current_pose.pose.orientation.z = 0.0;
    current_pose.pose.orientation.w = 0.0;

    timer = this->create_wall_timer(std::chrono::milliseconds(1000 / update_frequency),std::bind(&CupPosePublisher::pose_publisher_callback, this));
}

CupPosePublisher::~CupPosePublisher() {}

void CupPosePublisher::pose_publisher_callback()
{
    current_pose.header.stamp = this->get_clock()->now();
    publisher->publish(current_pose);
}

void CupPosePublisher::command_callback(const std_msgs::msg::String & command)
{
    std::stringstream ss(command.data);
    std::string item;
    std::vector<float> position;

    while (std::getline(ss, item, ','))
    {
        position.push_back(std::stof(item));
    }
//checking if its valid
    if (position.size() == 3)
    {
        current_pose.pose.position.x = position[0];
        current_pose.pose.position.y = position[1];
        current_pose.pose.position.z = position[2];
    }
}
