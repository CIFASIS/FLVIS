#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd src/FLVIS/3rdPartLib/
chmod +x install3rdPartLib_root.sh
sync
./install3rdPartLib_root.sh
cd $CATKIN_WS
catkin config --extend /opt/ros/$ROS_DISTRO \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DLOG_TRACKING_TIMESTAMPS=ON
catkin build
