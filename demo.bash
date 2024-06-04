#!/bin/bash

. install/setup.sh
ros2 launch robot_arm_sim simulation.launch.py &

sleep 5

ros2 topic pub --once /command std_msgs/msg/String '{data: "#5P1300T2000\r"}'
sleep 4

ros2 topic pub --once /command std_msgs/msg/String '{data: "#2P1700T2000\r"}'
sleep 4

ros2 topic pub --once /command std_msgs/msg/String '{data: "#1P1300T2000\r"}'
sleep 4

ros2 topic pub --once /command std_msgs/msg/String '{data: "#5P1600T2000\r"}'
sleep 4

ros2 topic pub --once /command std_msgs/msg/String '{data: "#2P1400T2000\r"}'
sleep 4

ros2 topic pub --once /command std_msgs/msg/String '{data: "#5P900T2000\r"}'
sleep 4