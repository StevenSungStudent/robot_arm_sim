from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    robot_arm_sim_path = FindPackageShare('robot_arm_sim')
    default_arm_model_path = PathJoinSubstitution([robot_arm_sim_path, 'urdf', 'lynxmotion_arm.urdf'])
    default_cup_model_path = PathJoinSubstitution([robot_arm_sim_path, 'urdf', 'cup.urdf'])
    default_rviz_config_path = PathJoinSubstitution([robot_arm_sim_path, 'rviz', 'urdf_config.rviz'])

    # These parameters are maintained for backwards compatibility
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    ld.add_action(gui_arg)
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)

    # This parameter has changed its meaning slightly from previous versions
    ld.add_action(DeclareLaunchArgument(name='arm_model', default_value=default_arm_model_path,
                                        description='Path to robot arm urdf file relative to robot_arm_sim package'))
    ld.add_action(DeclareLaunchArgument(name='cup_model', default_value=default_cup_model_path,
                                        description='Path to cup urdf file relative to robot_arm_sim package'))


    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'robot_arm_sim',
            'urdf_package_path': LaunchConfiguration('arm_model'),
            'rviz_config': LaunchConfiguration('rvizconfig'),
            'jsp_gui': LaunchConfiguration('gui')}.items()
    ))


    ld.add_action(Node(
        package="robot_arm_sim",
        executable="robot_arm_sim",
        name="robot_arm_sim"
    ))

    ld.add_action(Node(
        package="robot_arm_sim",
        executable="cup_pose_publisher",
        name="cup_pose_publisher"
    ))

    return ld
