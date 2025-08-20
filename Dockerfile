# Base image
FROM osrf/ros:noetic-desktop

# Tell the container to use the C.UTF-8 locale for its language settings
ENV LANG C.UTF-8

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]

# Install Gazebo 11 and other dependencies
RUN apt-get update && apt-get install -y \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-gazebo-ros-control \
  ros-noetic-ros-control \
  ros-noetic-ros-controllers \
  ros-noetic-joint-state-publisher \
  ros-noetic-robot-state-publisher \
  ros-noetic-robot-localization \
  ros-noetic-xacro \
  ros-noetic-tf2-ros \
  ros-noetic-tf2-tools \
  #ros-noetic-rmw-cyclonedds-cpp \
  python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/*

# Create workspace and download simulation repository

RUN source /opt/ros/noetic/setup.bash \
 && mkdir -p /catkin_ws/src

# Set up the catkin workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /tmp
RUN git clone --depth=1 https://github.com/mathrosas/ros1_testing.git \
 && mv ros1_testing/tortoisebot /catkin_ws/src/ \
 && mv ros1_testing/tortoisebot_waypoints /catkin_ws/src/ \
 && rm -rf ros1_testing

RUN /bin/bash -c "chmod +x /catkin_ws/src/tortoisebot_waypoints/src/tortoisebot_action_server.py"
RUN /bin/bash -c "chmod +x /catkin_ws/src/tortoisebot_waypoints/test/waypoints_test.test"

# Build the Colcon workspace and ensure it's sourced
RUN source /opt/ros/noetic/setup.bash \
 && cd /catkin_ws \
 && catkin_make
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# Set up a workspace directory
WORKDIR /catkin_ws/

# Set environment variables
ENV DISPLAY=:1
ENV GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}
ENV ROS_DOMAIN_ID=1
#ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# We want /bin/bash to execute our /entrypoint.sh when container starts
#CMD ["/entrypoint.sh"]
