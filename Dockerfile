FROM osrf/ros:noetic-desktop-full

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential

# Initialize rosdep
RUN rosdep init && rosdep update

# Create workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src

# Clone your repository
RUN git clone https://github.com/mathrosas/ros1_testing.git

# Install dependencies
RUN rosdep install --from-paths . --ignore-src -y

# Build the workspace
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Add setup to bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# Test entrypoint
CMD ["/bin/bash", "-c", "source /catkin_ws/devel/setup.bash && \
     roslaunch tortoisebot_gazebo tortoisebot_playground.launch gui:=false & \
     sleep 20 && \
     rostest tortoisebot_waypoints waypoints_test.test"]