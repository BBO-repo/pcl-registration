FROM osrf/ros:humble-desktop-full

WORKDIR /workspace

RUN apt update && apt -y dist-upgrade 
RUN apt install -y nano git wget software-properties-common \
  ros-humble-ros2-control ros-humble-ros2-controllers\
  ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-joint-state-publisher-gui \
  ros-humble-velodyne-simulator ros-humble-imu-tools \ 
  && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository -y ppa:borglab/gtsam-release-4.1 && \
  apt install -y libgtsam-dev libgtsam-unstable-dev

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
