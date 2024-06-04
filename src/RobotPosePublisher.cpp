#include "RobotPosePublisher.hpp"
#include <regex>
#include <vector>

enum joints_index{
    base = 0,
    turret = 1,
    upperarm = 2,
    forearm = 3,
    wrist = 4,
    gripperleft = 5,
    gripperright = 6
};

RobotPosePublisher::RobotPosePublisher() : 
    Node("pose_publisher"), max_pwm_(2500), min_pwm_(500), update_frequency_(10)
{
    publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    joint_states.name = {"base_link2turret", "turret2upperarm", "upperarm2forearm", "forearm2wrist", "wrist2hand", "gripper_left2hand", "gripper_right2hand"};
    joint_states.position = {0,0,0,0,0,0,0};
    joint_states.velocity = {0,0,0,0,0,0,0};
    delta_angle = {0,0,0,0,0,0,0};

    current_joint_states = joint_states;

    subscription = this->create_subscription<std_msgs::msg::String>("command", 10, std::bind(&RobotPosePublisher::command_callback, this, std::placeholders::_1));
    timer = this->create_wall_timer(std::chrono::milliseconds(update_frequency_),std::bind(&RobotPosePublisher::joint_publisher_callback, this));
    
}

RobotPosePublisher::~RobotPosePublisher()
{

}

void RobotPosePublisher::joint_publisher_callback(){

    for(unsigned short i = 0; i < joint_states.position.size(); ++i){
        if(current_joint_states.velocity.at(i) < joint_states.velocity.at(i)){

            current_joint_states.position.at(i) += (delta_angle.at(i)) / (joint_states.velocity.at(i) / update_frequency_);
            current_joint_states.velocity.at(i) += update_frequency_;
        }
    }
    current_joint_states.header.stamp = this->get_clock()->now();
    publisher->publish(current_joint_states);

}

void RobotPosePublisher::command_callback(const std_msgs::msg::String & command){ 
    const std::regex regex_pattern_("#(\\d+)P(\\d+)T(\\d+)\r");
    std::smatch match;

    if (std::regex_search(command.data, match, regex_pattern_))
    {
        joint_states.velocity.at(std::stoi(match.str(1))) = std::stoi(match.str(3));

        if(std::stoi(match.str(1)) == gripperleft){
           joint_states.position.at(gripperleft) = PWM_to_meter(std::stoi(match.str(2)));
           joint_states.position.at(gripperright) = joint_states.position.at(gripperleft);
           joint_states.velocity.at(gripperright) = joint_states.velocity.at(gripperleft);

        }else{
            joint_states.position.at(std::stoi(match.str(1))) = PWM_to_angle(std::stoi(match.str(2)));
        }

        for(unsigned long i = 0; i < joint_states.position.size(); ++i){
            delta_angle.at(i) = joint_states.position.at(i) - current_joint_states.position.at(i);
            
        }
        current_joint_states.velocity = {0,0,0,0,0,0,0};
    }
}

double RobotPosePublisher::PWM_to_angle(const long& value) const{
    const double MULTIPLIER = 11.11111111112;
    const double PI_VALUE = 3.14159265359;

    return (((value - 1500) / MULTIPLIER) * PI_VALUE / 180);
}

double RobotPosePublisher::PWM_to_meter(const long& value) const{
    const double max_ = 0.015;
    const double min_ = -0.015;

    return (value - min_pwm_) * (max_ - min_) / (max_pwm_ - min_pwm_) + min_;
}
