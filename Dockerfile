# Define parent image
FROM ros:kinetic-perception

# Set environment and working directory
ENV CATKIN_WS=/root/catkin_ws
WORKDIR $CATKIN_WS
ENV DEBIAN_FRONTEND noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    libopencv-dev \
    libqt4-dev \
    libsuitesparse-dev \
    qt4-qmake \
    python-catkin-tools \
    ros-kinetic-rviz && \
    rm -rf /var/lib/apt/lists/*

# Copy files
COPY . src/FLVIS/

# Build SLAM system
RUN cd src/FLVIS/3rdPartLib/ && \
    ./install3rdPartLib.sh && \
    cd $CATKIN_WS && \
    catkin config --extend /opt/ros/$ROS_DISTRO \
      --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build && \
    sed -i '/exec "$@"/i source ~/catkin_ws/devel/setup.bash' /ros_entrypoint.sh
