#!/bin/bash
set -euo pipefail
set -x

WS=/simulation_ws
source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash

Xvfb :1 -screen 0 1280x1024x24 >/tmp/xvfb.log 2>&1 &
XVFB_PID=$!
export DISPLAY=:1

echo "[$(date '+%F %T')] Launching TortoiseBot world..."
roslaunch tortoisebot_gazebo tortoisebot_playground.launch gui:=false \
  >/tmp/roslaunch.log 2>&1 &
GAZEBO_PID=$!

echo "[$(date '+%F %T')] Waiting for ROS master..."
until rosnode list >/dev/null 2>&1; do sleep 1; done

echo "[$(date '+%F %T')] Waiting for Gazebo..."
until rostopic list | grep -E '^/gazebo($|/)' >/dev/null 2>&1 || rostopic list | grep -q '^/clock$'; do sleep 1; done

echo "[$(date '+%F %T')] Starting Waypoints Action Server..."
rosrun tortoisebot_waypoints tortoisebot_action_server.py >/tmp/action_server.log 2>&1 &
AS_PID=$!

echo "[$(date '+%F %T')] Waiting for /tortoisebot_as..."
for i in {1..120}; do
  if rostopic list | grep -q '^/tortoisebot_as'; then break; fi
  sleep 1
done

echo "[$(date '+%F %T')] Running rostest..."
set +e
rostest tortoisebot_waypoints waypoints_test.test --reuse-master
TEST_RESULT=$?
set -e
echo "[$(date '+%F %T')] TEST RESULT: $TEST_RESULT"

echo "[$(date '+%F %T')] Shutting down..."
kill $AS_PID 2>/dev/null || true
kill $GAZEBO_PID 2>/dev/null || true
kill $XVFB_PID 2>/dev/null || true
pkill -9 -f gzserver || true
pkill -9 -f gzclient || true

exit $TEST_RESULT
