#!/usr/bin/env bash
# Capture a live "screenshot" of the running stack: node graph, topic list, and
# a short sample of the key topics. Output is written to
# docs/reproduction/system_snapshot.txt.
set -e
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -i miniconda | grep -v -i anaconda | paste -sd:)
source /opt/ros/humble/setup.bash
REPO=/home/lry/mars_navigation
source "$REPO/ros2/install/setup.bash"
OUT="$REPO/docs/reproduction/system_snapshot.txt"

ros2 launch mars_navigation_ros2 smoke_stack.launch.py \
    > /tmp/snapshot_stack.log 2>&1 &
LAUNCH_PID=$!
sleep 8   # let nodes come up and topics connect

{
  echo "==================== ros2 node list ===================="
  ros2 node list
  echo
  echo "==================== ros2 topic list ===================="
  ros2 topic list
  echo
  echo "==================== /navigation_mode (last msg) ===================="
  timeout 3 ros2 topic echo /navigation_mode std_msgs/msg/String --once 2>/dev/null || echo "(no msg within 3s)"
  echo
  echo "==================== /pose_with_covariance (last msg) ===================="
  timeout 3 ros2 topic echo /pose_with_covariance geometry_msgs/msg/PoseWithCovarianceStamped --once 2>/dev/null || echo "(no msg within 3s)"
  echo
  echo "==================== /map_server/global_map (header) ===================="
  timeout 3 ros2 topic echo /map_server/global_map nav_msgs/msg/OccupancyGrid --once --field info 2>/dev/null || echo "(no msg within 3s)"
} > "$OUT" 2>&1

kill -INT "$LAUNCH_PID" 2>/dev/null
sleep 3
kill -TERM "$LAUNCH_PID" 2>/dev/null
pkill -f "install/mars_navigation_ros2" 2>/dev/null
echo "wrote $OUT"
