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
ENV WS=/simulation_ws
RUN mkdir -p $WS/src
WORKDIR $WS

# Your repo with tortoisebot + tortoisebot_waypoints
ARG REPO_URL="https://github.com/mathrosas/ros1_testing.git"
ARG REPO_BRANCH="main"
RUN source /opt/ros/noetic/setup.bash \
 && cd $WS/src \
 && git clone --branch "$REPO_BRANCH" --depth 1 "$REPO_URL" ros1_testing

# Ensure Python3 shebang + exec bit for Noetic
RUN if [ -f "$WS/src/ros1_testing/tortoisebot_waypoints/scripts/tortoisebot_action_server.py" ]; then \
      sed -i '1s|python$|python3|' "$WS/src/ros1_testing/tortoisebot_waypoints/scripts/tortoisebot_action_server.py" || true; \
      chmod +x "$WS/src/ros1_testing/tortoisebot_waypoints/scripts/tortoisebot_action_server.py"; \
    fi

# Build
RUN source /opt/ros/noetic/setup.bash && cd $WS && catkin_make

# Persist env
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
 && echo "source $WS/devel/setup.bash" >> /root/.bashrc

ENV DISPLAY=:1
WORKDIR $WS

COPY entrypoint.sh $WS/entrypoint.sh
RUN chmod +x $WS/entrypoint.sh
ENTRYPOINT ["/simulation_ws/entrypoint.sh"]
