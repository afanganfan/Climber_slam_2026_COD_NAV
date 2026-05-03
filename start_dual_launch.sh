#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="/home/nucshao/Climber_slam_2026_NAV"
CODE_WORKSPACE_DIR="/home/nucshao/Climber_slam_2026_code"
ROS_SETUP_FILE="${ROS_SETUP_FILE:-/opt/ros/humble/setup.bash}"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"

if [[ ! -f "$ROS_SETUP_FILE" ]]; then
  echo "[ERROR] ROS setup file not found: $ROS_SETUP_FILE"
  exit 1
fi

if [[ ! -f "$WORKSPACE_DIR/install/setup.bash" ]]; then
  echo "[ERROR] Workspace setup file not found: $WORKSPACE_DIR/install/setup.bash"
  echo "[HINT] Please build first: cd $WORKSPACE_DIR && colcon build"
  exit 1
fi

if [[ ! -f "$CODE_WORKSPACE_DIR/install/setup.bash" ]]; then
  echo "[ERROR] Code workspace setup file not found: $CODE_WORKSPACE_DIR/install/setup.bash"
  echo "[HINT] Please build first: cd $CODE_WORKSPACE_DIR && colcon build"
  exit 1
fi

mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR

# 加载 ROS2 与工作区环境。
# 注意：ROS setup 脚本内部会访问未定义变量，需临时关闭 nounset(-u)。
set +u
source "$ROS_SETUP_FILE"
source "$WORKSPACE_DIR/install/setup.bash"
source "$CODE_WORKSPACE_DIR/install/setup.bash"
set -u

PIDS=()

cleanup() {
  # 只在有后台进程时执行清理
  if [[ ${#PIDS[@]} -gt 0 ]]; then
    echo ""
    echo "[INFO] Stopping launched processes..."
    for pid in "${PIDS[@]}"; do
      if kill -0 "$pid" 2>/dev/null; then
        kill "$pid" 2>/dev/null || true
      fi
    done
    wait || true
  fi
}

trap cleanup INT TERM EXIT

cd "$WORKSPACE_DIR"

ros2 launch livox_ros_driver2 rviz_MID360_launch.py &
PIDS+=("$!")

# 给第一个 launch 少量启动时间，避免日志相互抢占
sleep 1

ros2 launch cod_bringup singlenav_launch.py &
PIDS+=("$!")

# 启动 COD 行为树前端 tactical_front
sleep 1
(
  cd "$CODE_WORKSPACE_DIR"
  ros2 launch cod_behavior cod_front_tactical.launch.py
) &
PIDS+=("$!")

wait
