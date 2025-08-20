FROM osrf/ros:noetic-desktop
SHELL ["/bin/bash","-c"]

# Deps
RUN apt-get update && apt-get install -y \
    git xvfb \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-ros-control ros-noetic-ros-controllers \
    ros-noetic-joint-state-publisher ros-noetic-robot-state-publisher \
    ros-noetic-robot-localization ros-noetic-xacro \
    ros-noetic-tf2-ros ros-noetic-tf2-tools \
    ros-noetic-rostest \
 && rm -rf /var/lib/apt/lists/*

# Workspace
ENV ROS_MASTER_URI=http://localhost:11311
RUN mkdir -p $/simulation_ws/src
WORKDIR $/simulation_ws

# Your repo with tortoisebot + tortoisebot_waypoints
ARG REPO_URL="https://github.com/mathrosas/ros1_testing.git"
ARG REPO_BRANCH="main"
RUN source /opt/ros/noetic/setup.bash \
 && cd /simulation_ws/src \
 && git clone --branch "$REPO_BRANCH" --depth 1 "$REPO_URL" ros1_testing

# Ensure Python3 shebang + exec bit for Noetic
RUN if [ -f "/simulation_ws/src/ros1_testing/tortoisebot_waypoints/scripts/tortoisebot_action_server.py" ]; then \
      sed -i '1s|python$|python3|' "/simulation_ws/src/ros1_testing/tortoisebot_waypoints/scripts/tortoisebot_action_server.py" || true; \
      chmod +x "/simulation_ws/src/ros1_testing/tortoisebot_waypoints/scripts/tortoisebot_action_server.py"; \
    fi

# Build
RUN source /opt/ros/noetic/setup.bash && cd /simulation_ws && catkin_make

# Persist env
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
 && echo "source /simulation_ws/devel/setup.bash" >> /root/.bashrc

ENV DISPLAY=:1
WORKDIR /simulation_ws

COPY entrypoint.sh /simulation_ws/entrypoint.sh
RUN chmod +x /simulation_ws/entrypoint.sh
ENTRYPOINT ["/simulation_ws/entrypoint.sh"]
