#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
