FROM osrf/ros:noetic-desktop-full-focal

# Tell the container to use the C.UTF-8 locale for its language settings
ENV LANG C.UTF-8

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Install required packages
RUN set -x \
    && apt-get update \
    && apt-get --with-new-pkgs upgrade -y \
    && apt-get install -y git \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-control \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-plugins \
    && rm -rf /var/lib/apt/lists/*

# Link python3 to python otherwise ROS scripts fail when using the OSRF contianer
RUN ln -sf /usr/bin/python3 /usr/bin/python

# Set up the catkin workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /tmp
RUN git clone --depth=1 https://github.com/mathrosas/ros1_testing.git \
 && mv ros1_testing/tortoisebot /catkin_ws/src/ \
 && mv ros1_testing/tortoisebot_waypoints /catkin_ws/src/ \
 && rm -rf ros1_testing

RUN /bin/bash -c "chmod +x /catkin_ws/src/tortoisebot_waypoints/src/tortoisebot_action_server.py"
RUN /bin/bash -c "chmod +x /catkin_ws/src/tortoisebot_waypoints/test/waypoints_test.test"

# Build
WORKDIR /catkin_ws

RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash && catkin_make"

# replace setup.bash in ros_entrypoint.sh
RUN sed -i 's|source "/opt/ros/\$ROS_DISTRO/setup.bash"|source "/catkin_ws/devel/setup.bash"|g' /ros_entrypoint.sh

# Cleanup
RUN rm -rf /root/.cache

# Start a bash shell when the container starts
CMD ["bash"]
