#!/usr/bin/env bash
# Isaac Sim 物理闭环对比实验（真实物理 + 真实渲染相机 + 真实翻车）。
#
# 与 run_experiments.sh 的区别：
#   - 导航栈用 isaac_stack.launch.py（删掉 pose_simulator / terrain_image_publisher）
#   - 位姿 / 相机图 / 翻车由 Isaac Sim（isaac/isaac_lunar_loop.py）经 DDS 提供
#
# 进程 B（导航栈）先起进入等待态，进程 A（Isaac）后起，加载地形/USD/Leo -> settle
# -> 发 /pose_with_covariance。脚本轮询该话题作为 "Isaac ready" 握手，随后启动
# evaluate 采集指标。结果写入 runs_isaac/<name>/：
#   stack.log      - 导航栈输出
#   isaac.log      - Isaac 物理/桥输出
#   evaluate.log   - 指标采集输出
#   metrics.json   - 通行时间/距离/成功率/各模式占比
#   trajectory.csv - (t, x, y, mode) 采样
#   isaac_physical.csv - 物理侧位姿/倾角日志
#
# Usage:
#   bash scripts/run_isaac_experiments.sh mode1   # mode1..3 / vlm / all

export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -i miniconda | grep -v -i anaconda | paste -sd:)

REPO=/home/lry/mars_navigation
ROS2_WS=$REPO/ros2
ISAAC_SIM_DIR=/home/lry/isaac-sim
source /opt/ros/humble/setup.bash
source "$ROS2_WS/install/setup.bash"

export ROS_DOMAIN_ID=0

GOAL_X=20.0
GOAL_Y=0.0
START_X=-20.0
START_Y=0.0
TIMEOUT=540.0               # Mode 3 需绕坑壁（~50 m @ 0.15 m/s ≈ 400 s + 物理侧滑损失），放宽到 540 s
RESULT_ROOT="$ROS2_WS/runs_isaac"
HEIGHTMAP="$REPO/docs/picture/terrain/lunar_terrain_hm.png"
ISAAC_SCRIPT="$REPO/isaac/isaac_lunar_loop.py"
SETTLE=3.0
ISAAC_MAX_TIME=$(( ${TIMEOUT%.*} + 120 ))   # Isaac 至少比 evaluate 超时多跑一会，避免先退出

run_one() {
    local name="$1"; shift
    local out="$RESULT_ROOT/$name"
    mkdir -p "$out"
    echo "==================== [$name] ===================="

    # 进程 B：导航栈（先起，各节点进入等待 pose 状态）。
    echo "[$name] launching nav stack..."
    ros2 launch mars_navigation_ros2 isaac_stack.launch.py "$@" \
        start_x:=$START_X start_y:=$START_Y goal_x:=$GOAL_X goal_y:=$GOAL_Y \
        heightmap:=$HEIGHTMAP \
        > "$out/stack.log" 2>&1 &
    local launch_pid=$!
    sleep 3

    # 进程 A：Isaac Sim（后起）。务必脱离 conda，避免 python.sh 的 conda 警告/冲突。
    # 关键：Isaac 自带 ROS2 bridge 用其捆绑的 humble 库（Python 3.11），与系统 ROS2 (3.10)
    # ABI 不兼容，须把捆绑 humble/lib 加入 LD_LIBRARY_PATH、把系统 ROS site-packages 从
    # PYTHONPATH 剔除（仅保留 IsaacLab source），并指定 ROS_DISTRO/RMW。
    echo "[$name] launching Isaac Sim..."
    (
        unset CONDA_PREFIX CONDA_DEFAULT_ENV
        unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH OLD_PYTHONPATH ROS_PACKAGE_PATH
        export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -iE 'miniconda|anaconda' | paste -sd:)
        # PYTHONPATH：剔除系统 ROS2 (3.10)，保留 IsaacLab source。
        export PYTHONPATH=$(echo "${PYTHONPATH:-}" | tr ':' '\n' | grep -v '/opt/ros' | grep -v '^$' | paste -sd:)
        # LD_LIBRARY_PATH：剔除系统 ROS / conda，前置捆绑 humble/lib。
        export LD_LIBRARY_PATH=$(echo "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v -iE '/opt/ros|miniconda|anaconda' | grep -v '^$' | paste -sd:)
        export LD_LIBRARY_PATH="$ISAAC_SIM_DIR/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH"
        export ROS_DOMAIN_ID=0
        export ROS_DISTRO=humble
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        "$ISAAC_SIM_DIR/python.sh" "$ISAAC_SCRIPT" \
            --settle "$SETTLE" --max-time "$ISAAC_MAX_TIME" --out "$out/isaac" \
            --record-dir "$out/record" --record-fps 30 \
            > "$out/isaac.log" 2>&1
    ) &
    local isaac_pid=$!

    # 握手：/pose_with_covariance 首现 = Isaac settle 完成，可开始计时采集。
    echo "[$name] waiting for Isaac pose handshake (up to 300s)..."
    # 显式给出消息类型，否则话题尚不存在时 ros2 topic echo 会因无法判定类型而立即失败。
    if ! timeout 300 ros2 topic echo /pose_with_covariance \
        geometry_msgs/msg/PoseWithCovarianceStamped --once \
        > /dev/null 2>&1; then
        echo "[$name] ERROR: Isaac pose handshake timed out; tail $out/isaac.log:" >&2
        tail -20 "$out/isaac.log" >&2
        pkill -f isaac_lunar_loop.py 2>/dev/null
        kill -INT "$isaac_pid" 2>/dev/null
        kill -INT "$launch_pid" 2>/dev/null
        return 1
    fi

    echo "[$name] evaluating..."
    ros2 run mars_navigation_ros2 evaluate --ros-args \
        -p goal_x:=$GOAL_X -p goal_y:=$GOAL_Y \
        -p timeout_s:=$TIMEOUT -p out_dir:=$out \
        > "$out/evaluate.log" 2>&1

    # 收尾：先杀 Isaac，再杀导航栈。
    pkill -f isaac_lunar_loop.py 2>/dev/null
    kill -INT "$isaac_pid" 2>/dev/null
    sleep 2
    kill -INT "$launch_pid" 2>/dev/null
    sleep 4
    kill -TERM "$launch_pid" 2>/dev/null
    pkill -f "install/mars_navigation_ros2" 2>/dev/null
    sleep 2

    # 合成视频（第三视角 + 第一视角），PNG 帧保留作为照片。
    for view in orbit cam; do
        if ls "$out/record/$view"/frame_*.png >/dev/null 2>&1; then
            ffmpeg -y -framerate 30 -i "$out/record/$view/frame_%06d.png" \
                -c:v libx264 -pix_fmt yuv420p -movflags +faststart \
                "$out/${view}.mp4" >/dev/null 2>&1 \
                && echo "[$name] wrote $out/${view}.mp4"
        fi
    done

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
