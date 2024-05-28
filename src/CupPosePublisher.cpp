#include "CupPosePublisher.hpp"

CupPosePublisher::CupPosePublisher() : Node("cup_pose_publisher"), update_frequency(10)
{
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    subscription = this->create_subscription<std_msgs::msg::String>("cup_command", 10, std::bind(&CupPosePublisher::command_callback, this, std::placeholders::_1));

    current_pose.header.frame_id = "base_link";
    current_pose.child_frame_id = "cup_link";
    current_pose.transform.translation.x = 0;
    current_pose.transform.translation.y = 0;
    current_pose.transform.translation.z = 0;

    timer = this->create_wall_timer(std::chrono::milliseconds(1000 / update_frequency),std::bind(&CupPosePublisher::pose_publisher_callback, this));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

CupPosePublisher::~CupPosePublisher() {}

void CupPosePublisher::pose_publisher_callback()
{    current_pose.header.stamp = this->get_clock()->now();

    tf_broadcaster_->sendTransform(current_pose);

    parse_transform_data();
    update_cup_position();
}

void CupPosePublisher::command_callback(const std_msgs::msg::String & command)
{

}

void CupPosePublisher::update_cup_position(){
    const double hand_width = 0.3;
    const double gripper_distance = 0.3;
    std::cout << "1: " << distance(current_pose.transform, gripper_left_position)  << std::endl;
    std::cout << "2: " << distance(current_pose.transform, hand_position)  << std::endl;
    if(distance(current_pose.transform, gripper_left_position) < gripper_distance && distance(current_pose.transform, gripper_right_position) < gripper_distance && distance(current_pose.transform, hand_position) < hand_width){
        std::cout << "zammmm" << std::endl;
    }
}

void CupPosePublisher::parse_transform_data(){
    geometry_msgs::msg::TransformStamped t_hand;
    geometry_msgs::msg::TransformStamped t_gripper_left;
    geometry_msgs::msg::TransformStamped t_gripper_right;

    try {
        t_hand = tf_buffer_->lookupTransform(
        "base_link", "hand",
        tf2::TimePointZero);
        hand_position = t_hand.transform;

        t_gripper_left = tf_buffer_->lookupTransform(
        "base_link", "gripper_left",
        tf2::TimePointZero);
        gripper_left_position = t_gripper_left.transform;

        t_gripper_right = tf_buffer_->lookupTransform(
        "base_link", "gripper_right",
        tf2::TimePointZero);
        gripper_right_position = t_gripper_right.transform;

    } catch (const tf2::TransformException & ex) {
        return;
    }
}

double CupPosePublisher::distance(geometry_msgs::msg::Transform point1, geometry_msgs::msg::Transform point2){
    return (sqrt(pow(point2.translation.x - point1.translation.x, 2) +  pow(point2.translation.y - point1.translation.y, 2) +  pow(point2.translation.z - point1.translation.z, 2) * 1.0));
}
