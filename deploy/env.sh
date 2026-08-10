#!/usr/bin/env bash
# Common environment setup for ROS2 rover services

source /opt/ros/jazzy/setup.bash
source ~/minix_ws/install/setup.bash

export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=0
