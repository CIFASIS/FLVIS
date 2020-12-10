FROM ros:kinetic-perception

ENV CATKIN_WS=/root/catkin_ws \
    FLVIS_ROOT=/root/catkin_ws/src/FLVIS/ \
    DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    libopencv-dev \
    libqt4-dev \
    libsuitesparse-dev \
    qt4-qmake \
    python-catkin-tools \
    ros-kinetic-rviz && \
    rm -rf /var/lib/apt/lists/*

COPY . $FLVIS_ROOT
COPY ./scripts $CATKIN_WS

WORKDIR $CATKIN_WS

RUN /bin/bash -c "chmod +x build.sh && chmod +x modify_entrypoint.sh && sync && ./build.sh && ./modify_entrypoint.sh"
