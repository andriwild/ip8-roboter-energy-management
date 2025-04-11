#!/bin/bash

if [ -z "$ROS2_DISTRO" ];
then
  ROS2_DISTRO="galactic"
fi

source /etc/ros/setup.bash

# load the ROS2 environment
source /opt/ros/${ROS2_DISTRO}/setup.bash
source /home/administrator/ip8-roboter-energy-management/install/setup.sh

export ROS_DOMAIN_ID=100
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export CYCLONEDDS_URI=file://$(catkin_find cpr_indoornav_base config/cyclone_dds.xml --first-only)

ros2 run csv_collector csv_collector -- "Total current (A)" "Battery Voltage (V)" "X" "Y" "Yaw" "Linear Velocity" "Angular Velocity"