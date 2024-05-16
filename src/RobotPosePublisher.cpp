#include "RobotPosePublisher.hpp"
#include "iostream"
#include <regex>
#include <vector>

RobotPosePublisher::RobotPosePublisher() : 
    Node("pose_publisher"), update_frequency(10)
{
    publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    joint_states.name = {"base_link2turret", "turret2upperarm", "upperarm2forearm", "forearm2wrist", "wrist2hand", "gripper_left2hand", "gripper_right2hand"};
    joint_states.position = {0,0,0,0,0,0,0};
    joint_states.velocity = {0,0,0,0,0,0,0};
    delta_angle = {0,0,0,0,0,0,0};

    current_joint_states = joint_states;

    subscription = this->create_subscription<std_msgs::msg::String>("command", 10, std::bind(&RobotPosePublisher::command_callback, this, std::placeholders::_1));
    timer = this->create_wall_timer(std::chrono::milliseconds(update_frequency),std::bind(&RobotPosePublisher::joint_publisher_callback, this));
}

RobotPosePublisher::~RobotPosePublisher()
{

}

void RobotPosePublisher::joint_publisher_callback(){
    current_joint_states.header.stamp = this->get_clock()->now();
    for(unsigned short i = 0; i < joint_states.position.size(); ++i){
        if(current_joint_states.velocity.at(i) < joint_states.velocity.at(i)){

            current_joint_states.position.at(i) += (delta_angle.at(i)) / (joint_states.velocity.at(i) / update_frequency);
            current_joint_states.velocity.at(i) += update_frequency;
        }else{
            current_joint_states = joint_states;
        }
    }
    publisher->publish(current_joint_states);
}

void RobotPosePublisher::command_callback(const std_msgs::msg::String & command){ 
    std::regex regex_pattern("#(\\d+)P(\\d+)T(\\d+)\r");
    std::smatch match;

    if (std::regex_search(command.data, match, regex_pattern));
    {
        joint_states.position.at(std::stoi(match.str(1))) = std::stoi(match.str(2));
        joint_states.velocity.at(std::stoi(match.str(1))) = std::stoi(match.str(3));
        current_joint_states.velocity = {0,0,0,0,0,0,0};
        for(unsigned long i = 0; i < joint_states.position.size(); ++i){
            delta_angle.at(i) = joint_states.position.at(i) - current_joint_states.position.at(i);
        }
    }


}
