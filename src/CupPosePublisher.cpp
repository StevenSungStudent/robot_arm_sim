#include "CupPosePublisher.hpp"

CupPosePublisher::CupPosePublisher() : Node("cup_pose_publisher"), update_frequency(10)
{
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    subscription = this->create_subscription<std_msgs::msg::String>("cup_command", 10, std::bind(&CupPosePublisher::command_callback, this, std::placeholders::_1));

    current_pose.header.frame_id = "base_link";
    current_pose.child_frame_id = "cup_link";
    current_pose.transform.translation.x = 0.3;
    current_pose.transform.translation.y = 0;
    current_pose.transform.translation.z = 0.05;

    timer = this->create_wall_timer(std::chrono::milliseconds(1000 / update_frequency),std::bind(&CupPosePublisher::pose_publisher_callback, this));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

CupPosePublisher::~CupPosePublisher() {}

void CupPosePublisher::pose_publisher_callback()
{    

    parse_transform_data();
    update_cup_position();
    
    current_pose.header.stamp = this->get_clock()->now();
    tf_broadcaster_->sendTransform(current_pose);


}

void CupPosePublisher::command_callback(const std_msgs::msg::String & command)
{

}

void CupPosePublisher::update_cup_position(){
    const double gripper_distance_ = 0.04;
    const double gripper_joint_offset_ = 0.02;
    const double cup_height_ = 0.08;
    const double cup_width_ = 0.03;

    static bool offset_set = false;
    static geometry_msgs::msg::TransformStamped offset;

    if( 
        distance_2d(gripper_left_position.translation.x, gripper_left_position.translation.y, gripper_right_position.translation.x, gripper_right_position.translation.y) <= cup_width_ + gripper_joint_offset_ && //Check if the grippers are closed enough to hold the cup.  
        distance_2d(current_pose.transform.translation.x, current_pose.transform.translation.y, gripper_left_position.translation.x, gripper_left_position.translation.y) < gripper_distance_ &&  //Check if the grippers are near the cup.  
        distance_2d(current_pose.transform.translation.x, current_pose.transform.translation.y, gripper_right_position.translation.x, gripper_right_position.translation.y) < gripper_distance_ &&
        std::fabs(current_pose.transform.translation.z - gripper_left_position.translation.z) < (cup_height_ / 2) + current_pose.transform.translation.z)   //Check if the grippers are at the correct height.
    {
        if(!offset_set){
            offset.transform.translation.x = current_pose.transform.translation.x - hand_position.translation.x;
            offset.transform.translation.y = current_pose.transform.translation.y - hand_position.translation.y;
            offset.transform.translation.z = current_pose.transform.translation.z - hand_position.translation.z;
            offset_set = true;
        };

        current_pose.transform.translation.x = hand_position.translation.x + offset.transform.translation.x;
        current_pose.transform.translation.y = hand_position.translation.y + offset.transform.translation.y;
        current_pose.transform.translation.z = hand_position.translation.z + offset.transform.translation.z;
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

double CupPosePublisher::distance_2d(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow((x2 - x1), 2) + std::pow((y2 - y1), 2));
}