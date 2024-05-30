#include "CupPosePublisher.hpp"

CupPosePublisher::CupPosePublisher() : Node("cup_pose_publisher"), update_frequency(10), cup_gripped(false)
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
    const double hand_width = 0.05;//TODO: make the it treat the cup like a cylinder instead of a ball.
    const double gripper_distance = 0.04;//TODO: make this make sense.
    const double cup_height = 0.08;
    const double cup_width = 0.03;

    static bool offset_set = false;
    static geometry_msgs::msg::TransformStamped offset;
    // std::cout << "1: " << distance(current_pose.transform, gripper_left_position)  << std::endl;
    // std::cout << "2: " << distance(current_pose.transform, hand_position)  << std::endl;
    // if(distance(current_pose.transform, gripper_left_position) < gripper_distance && distance(current_pose.transform, gripper_right_position) < gripper_distance && distance(current_pose.transform, hand_position) < hand_width){
    //     std::cout << "zammmm" << std::endl;
    //     cup_gripped = true;
        
    //     current_pose.transform.translation.x += hand_position.translation.x - previous_hand_position.translation.x;
    //     current_pose.transform.translation.y += hand_position.translation.y - previous_hand_position.translation.y;
    //     current_pose.transform.translation.z += hand_position.translation.z - previous_hand_position.translation.z;
    //     current_pose.transform.rotation.x += hand_position.rotation.x - previous_hand_position.rotation.x;
    //     current_pose.transform.rotation.y += hand_position.rotation.y - previous_hand_position.rotation.y;
    //     current_pose.transform.rotation.z += hand_position.rotation.z - previous_hand_position.rotation.z;
    //     current_pose.transform.rotation.w += hand_position.rotation.w - previous_hand_position.rotation.w;
    //     std::cout << "x: " << current_pose.transform.translation.x  << std::endl;
    //     std::cout << "x prev: " << previous_hand_position.translation.x << std::endl;
    //     std::cout << "x now : " << hand_position.translation.x << std::endl;
    //     std::cout << "y: " << current_pose.transform.translation.y << std::endl;
    //     std::cout << "y prev: " << previous_hand_position.translation.y << std::endl;
    //     std::cout << "y now : " << hand_position.translation.y << std::endl;
    //     std::cout << "z: " << current_pose.transform.translation.z  << std::endl;
    //     std::cout << "z prev: " << previous_hand_position.translation.z << std::endl;
    //     std::cout << "z now : " << hand_position.translation.z << std::endl;
    // }else{
    //     cup_gripped = false;
    // }

    // if(distance_2d(current_pose.transform.translation.x, current_pose.transform.translation.y, gripper_left_position.translation.x, gripper_left_position.translation.y) < gripper_distance && distance_2d(current_pose.transform.translation.x, current_pose.transform.translation.y, gripper_right_position.translation.x, gripper_right_position.translation.y) < gripper_distance  && std::fabs(current_pose.transform.translation.y - gripper_left_position.translation.y) < (cup_height / 2) + current_pose.transform.translation.y && distance(current_pose.transform, hand_position) < hand_width){
    //     std::cout << "zammmm" << std::endl;
    //     cup_gripped = true;

    //     if(current_pose.transform.rotation.z + hand_position.rotation.z - previous_hand_position.rotation.z > 0.05){
    //         current_pose.transform.translation.x += hand_position.translation.x - previous_hand_position.translation.x;
    //         current_pose.transform.translation.y += hand_position.translation.y - previous_hand_position.translation.y;
    //         current_pose.transform.translation.z += hand_position.translation.z - previous_hand_position.translation.z;
    //         current_pose.transform.rotation.x += hand_position.rotation.x - previous_hand_position.rotation.x;
    //         current_pose.transform.rotation.y += hand_position.rotation.y - previous_hand_position.rotation.y;
    //         current_pose.transform.rotation.z += hand_position.rotation.z - previous_hand_position.rotation.z;
    //         current_pose.transform.rotation.w += hand_position.rotation.w - previous_hand_position.rotation.w;
    //         std::cout << "x: " << current_pose.transform.translation.x  << std::endl;
    //         std::cout << "x prev: " << previous_hand_position.translation.x << std::endl;
    //         std::cout << "x now : " << hand_position.translation.x << std::endl;
    //         std::cout << "y: " << current_pose.transform.translation.y << std::endl;
    //         std::cout << "y prev: " << previous_hand_position.translation.y << std::endl;
    //         std::cout << "y now : " << hand_position.translation.y << std::endl;
    //         std::cout << "z: " << current_pose.transform.translation.z  << std::endl;
    //         std::cout << "z prev: " << previous_hand_position.translation.z << std::endl;
    //         std::cout << "z now : " << hand_position.translation.z << std::endl;
    //     }
    // }else{
    //     cup_gripped = false;
    // }

    if( distance_2d(gripper_left_position.translation.x, gripper_left_position.translation.y, gripper_right_position.translation.x, gripper_right_position.translation.y) <= cup_width + 0.02 && distance_2d(current_pose.transform.translation.x, current_pose.transform.translation.y, gripper_left_position.translation.x, gripper_left_position.translation.y) < gripper_distance && distance_2d(current_pose.transform.translation.x, current_pose.transform.translation.y, gripper_right_position.translation.x, gripper_right_position.translation.y) < gripper_distance  && std::fabs(current_pose.transform.translation.z - gripper_left_position.translation.z) < (cup_height / 2) + current_pose.transform.translation.z){
        std::cout << "zammmm" << std::endl;
        cup_gripped = true;

        if(!offset_set){
            offset.transform.translation.x = current_pose.transform.translation.x - hand_position.translation.x;
            offset.transform.translation.y = current_pose.transform.translation.y - hand_position.translation.y;
            offset.transform.translation.z = current_pose.transform.translation.z - hand_position.translation.z;
            offset_set = true;
        };
        // current_pose.transform.rotation.y += hand_position.rotation.y - previous_hand_position.rotation.y;
        // current_pose.transform.rotation.z += hand_position.rotation.z - previous_hand_position.rotation.z;
        // current_pose.transform.rotation.w += hand_position.rotation.w - previous_hand_position.rotation.w;

        // current_pose.transform.translation.x += hand_position.translation.x - previous_hand_position.translation.x;
        // current_pose.transform.translation.y += hand_position.translation.y - previous_hand_position.translation.y;
        // current_pose.transform.translation.z += hand_position.translation.z - previous_hand_position.translation.z;
        // current_pose.transform.rotation.x += hand_position.rotation.x - previous_hand_position.rotation.x;
        // current_pose.transform.rotation.y += hand_position.rotation.y - previous_hand_position.rotation.y;
        // current_pose.transform.rotation.z += hand_position.rotation.z - previous_hand_position.rotation.z;
        // current_pose.transform.rotation.w += hand_position.rotation.w - previous_hand_position.rotation.w;

        current_pose.transform.translation.x = hand_position.translation.x + offset.transform.translation.x;
        current_pose.transform.translation.y = hand_position.translation.y + offset.transform.translation.y;
        current_pose.transform.translation.z = hand_position.translation.z + offset.transform.translation.z;
        // current_pose.transform.rotation.x += hand_position.rotation.x - previous_hand_position.rotation.x;
        // current_pose.transform.rotation.y += hand_position.rotation.y - previous_hand_position.rotation.y;
        // current_pose.transform.rotation.z += hand_position.rotation.z - previous_hand_position.rotation.z;
        // current_pose.transform.rotation.w += hand_position.rotation.w - previous_hand_position.rotation.w;

    }else{
        cup_gripped = false;
    }

    previous_hand_position = hand_position;
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