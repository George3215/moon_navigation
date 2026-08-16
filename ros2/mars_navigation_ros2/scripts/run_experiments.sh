#!/usr/bin/env bash
# Headless single-mode / multi-mode traversal experiments.
#
# Launches the full ROS 2 stack, runs the metric collector ``evaluate`` against
# it, and tears everything down. Produces, per run, under ``runs/<name>/``:
#   stack.log     - full stack output
#   evaluate.log  - metric collector output
#   metrics.json  - traversal time / distance / success / per-mode breakdown
#   trajectory.csv- (t, x, y, mode) samples
#
# Usage:
#   bash scripts/run_experiments.sh mode1   # single mode 1..3, or vlm, or all

export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -i miniconda | grep -v -i anaconda | paste -sd:)

REPO=/home/lry/mars_navigation
ROS2_WS=$REPO/ros2
source /opt/ros/humble/setup.bash
source "$ROS2_WS/install/setup.bash"

GOAL_X=20.0
GOAL_Y=0.0
START_X=-20.0
START_Y=0.0
TIMEOUT=240.0               # per-run navigation timeout (s)
RESULT_ROOT="$ROS2_WS/runs"
# Heightmap to feed the stack (override with e.g. HEIGHTMAP=.../lunar_terrain_hm.png).
HEIGHTMAP="${HEIGHTMAP:-$REPO/docs/picture/terrain/mixed_terrain_hm.png}"

run_one() {
    local name="$1"; shift
    local out="$RESULT_ROOT/$name"
    mkdir -p "$out"
    echo "==================== [$name] ===================="

    echo "[$name] launching stack..."
    ros2 launch mars_navigation_ros2 smoke_stack.launch.py "$@" \
        start_x:=$START_X start_y:=$START_Y goal_x:=$GOAL_X goal_y:=$GOAL_Y \
        heightmap:=$HEIGHTMAP \
        > "$out/stack.log" 2>&1 &
    local launch_pid=$!

    # Wait for the launch to spawn and start publishing the pose.  Keep this
    # short so the evaluator captures the (nearly) full traversal.
    sleep 3

    echo "[$name] evaluating..."
    ros2 run mars_navigation_ros2 evaluate --ros-args \
        -p goal_x:=$GOAL_X -p goal_y:=$GOAL_Y \
        -p timeout_s:=$TIMEOUT -p out_dir:=$out \
        > "$out/evaluate.log" 2>&1

    # Tear down: SIGINT the launch, then clean up any stragglers.
    kill -INT "$launch_pid" 2>/dev/null
    sleep 4
    kill -TERM "$launch_pid" 2>/dev/null
    pkill -f "install/mars_navigation_ros2" 2>/dev/null
    sleep 2

    echo "[$name] metrics:"
    if [ -f "$out/metrics.json" ]; then
        cat "$out/metrics.json"
    else
        echo "  (no metrics.json written)"
        tail -5 "$out/evaluate.log"
    fi
    echo ""
}

case "${1:-all}" in
    mode1) run_one mode1 use_vlm:=false fixed_mode:=1 ;;
    mode2) run_one mode2 use_vlm:=false fixed_mode:=2 ;;
    mode3) run_one mode3 use_vlm:=false fixed_mode:=3 ;;
    vlm)   run_one vlm use_vlm:=true ;;
    all)
        run_one mode1 use_vlm:=false fixed_mode:=1
        run_one mode2 use_vlm:=false fixed_mode:=2
        run_one mode3 use_vlm:=false fixed_mode:=3
        run_one vlm use_vlm:=true
        ;;
    *)
        echo "usage: $0 {mode1|mode2|mode3|vlm|all}" >&2
        exit 1
        ;;
esac
