#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd src/FLVIS/3rdPartLib/
./install3rdPartLib_root.sh
cd $CATKIN_WS
catkin config --extend /opt/ros/$ROS_DISTRO \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
