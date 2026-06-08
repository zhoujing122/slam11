#!/usr/bin/env bash
# D100 实机 SLAM 一键启动 tmux session
#
# 用法:
#   bash src/lightning-lm/d100_slam_session.sh [config]
#
#   config: SLAM yaml 名（不含路径），可选
#       d100_slam_back.yaml   建图（默认）
#       d100_loc.yaml         定位
#
# 环境变量:
#   ROS2_WS              工作空间路径（默认 /ros2_ws）
#   D100_SESSION         tmux session 名（默认 d100）
#   D100_LAUNCH_ARGS     额外传给 launch 的参数
#                        例如: "start_lidar_driver:=false start_rviz:=false"
#
# Bag replay 用法（离线复盘录的 bag）:
#   1. 启动 tmux，关掉驱动和控制器，打开 sim_time:
#        D100_LAUNCH_ARGS="use_sim_time:=true start_lidar_driver:=false start_controllers:=false" \
#            bash d100_slam_session.sh d100_slam_back.yaml
#   2. 另开终端 replay bag，必须带 --clock:
#        ros2 bag play --clock <bag_path>
#   注意: use_sim_time 只透传给 robot_state_publisher / imu_converter /
#         pointcloud_merger / slam_tf_bridge / rviz，控制器栈和 rslidar 驱动不受影响。
#
# 退出/重启:
#   bash d100_slam_session.sh -k    # 杀掉已有 session
#   tmux attach -t d100             # 重新进入
#   tmux kill-session -t d100       # 手动销毁
#
# 窗口布局:
#   1 stack     : ros2_control + 三路雷达 + IMU 转换 + 融合 + RViz
#   2 slam      : Lightning-LM run_slam_online
#   3 monitor   : 4 分屏 — topic hz / topic list / tf monitor / 空闲 shell
#   4 bag       : 预填 ros2 bag record 命令（不自动按回车）
#   5 shell     : 空闲 shell，留给 save_map / 临时调试

set -euo pipefail

SESSION="${D100_SESSION:-d100}"
ROS2_WS="${ROS2_WS:-/ros2_ws}"
SLAM_CONFIG="${1:-d100_slam_back.yaml}"
LAUNCH_ARGS="${D100_LAUNCH_ARGS:-}"

# Lightning-LM 起来前等 stack 起完整的秒数
STACK_WARMUP_S=10

# ---------------------------------------------------------------------------
# 选项: -k 仅杀 session
# ---------------------------------------------------------------------------
if [[ "${1:-}" == "-k" || "${1:-}" == "--kill" ]]; then
    tmux kill-session -t "$SESSION" 2>/dev/null && echo "killed session: $SESSION" || echo "no session named $SESSION"
    exit 0
fi

# ---------------------------------------------------------------------------
# Pre-flight 检查
# ---------------------------------------------------------------------------
command -v tmux >/dev/null || { echo "ERR: tmux not installed"; exit 1; }

if [[ ! -d "$ROS2_WS" ]]; then
    echo "ERR: ROS2_WS=$ROS2_WS not found. Set ROS2_WS env var to your workspace path."
    exit 1
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "session '$SESSION' already exists. attach with: tmux attach -t $SESSION"
    echo "or kill it first: bash $0 -k"
    exit 1
fi

SLAM_CONFIG_PATH="src/lightning-lm/config/$SLAM_CONFIG"
if [[ ! -f "$ROS2_WS/$SLAM_CONFIG_PATH" ]]; then
    echo "WARN: $ROS2_WS/$SLAM_CONFIG_PATH not found. Lightning-LM will fail to start."
    echo "     check filename or set SLAM_CONFIG arg."
fi

# UDP buffer guard：三路 Airy ~30 MB/s 总数据率，default 208KB rmem 会丢包。
# 在 host 上一次性配置 /etc/sysctl.d/99-d100-lidar.conf 把 rmem_max 提到 32MB。
# 这里只检测 + 告警，不自动 sudo（脚本不应在无人值守时索要密码）。
RMEM_MAX=$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)
if (( RMEM_MAX < 16777216 )); then
    echo ""
    echo "WARN: net.core.rmem_max=${RMEM_MAX} (~$((RMEM_MAX/1024)) KB)，三路 LiDAR 会丢 UDP 包"
    echo "      在 host 上执行（一次性，持久化）："
    echo "        sudo tee /etc/sysctl.d/99-d100-lidar.conf > /dev/null <<EOF"
    echo "        net.core.rmem_max=33554432"
    echo "        net.core.rmem_default=33554432"
    echo "        EOF"
    echo "        sudo sysctl --system"
    echo ""
fi

echo "starting tmux session '$SESSION' in $ROS2_WS"
echo "  SLAM config : $SLAM_CONFIG"
echo "  launch args : ${LAUNCH_ARGS:-<none>}"

# ---------------------------------------------------------------------------
# 公共 init: 每个 pane 起来后先 source 一遍环境
# ---------------------------------------------------------------------------
COMMON_INIT="cd $ROS2_WS && source /opt/ros/humble/setup.bash && [[ -f install/setup.bash ]] && source install/setup.bash"

# 创建 session 同时创建第一个 window
tmux new-session -d -s "$SESSION" -n "stack" -x 220 -y 50

# ---------------------------------------------------------------------------
# Window 1: stack — 主 launch
# ---------------------------------------------------------------------------
tmux send-keys -t "$SESSION:stack" "$COMMON_INIT" C-m
tmux send-keys -t "$SESSION:stack" "echo '--- D100 stack: ros2_control + 3x rslidar + imu_converter + merger + RViz ---'" C-m
tmux send-keys -t "$SESSION:stack" \
    "ros2 launch d100_description d100_slam_real.launch.py $LAUNCH_ARGS" C-m

# ---------------------------------------------------------------------------
# Window 2: slam — Lightning-LM（延迟启动，等 stack warmup）
# ---------------------------------------------------------------------------
tmux new-window -t "$SESSION" -n "slam"
tmux send-keys -t "$SESSION:slam" "$COMMON_INIT" C-m
tmux send-keys -t "$SESSION:slam" \
    "echo '--- waiting ${STACK_WARMUP_S}s for stack to come up before launching Lightning-LM ---'" C-m
tmux send-keys -t "$SESSION:slam" "sleep $STACK_WARMUP_S" C-m
tmux send-keys -t "$SESSION:slam" \
    "ros2 run lightning run_slam_online --config $SLAM_CONFIG_PATH" C-m

# ---------------------------------------------------------------------------
# Window 3: monitor — 4 分屏检查 topic / tf
# ---------------------------------------------------------------------------
tmux new-window -t "$SESSION" -n "monitor"

# pane 0 (左上): merged 点云频率
tmux send-keys -t "$SESSION:monitor" "$COMMON_INIT" C-m
tmux send-keys -t "$SESSION:monitor" \
    "echo '[merged points hz]' && sleep $STACK_WARMUP_S && ros2 topic hz /merged/LIDAR/POINTS" C-m

# pane 1 (右上): IMU 频率
tmux split-window -t "$SESSION:monitor" -h
tmux send-keys -t "$SESSION:monitor" "$COMMON_INIT" C-m
tmux send-keys -t "$SESSION:monitor" \
    "echo '[imu hz]' && sleep $STACK_WARMUP_S && ros2 topic hz /imu/data" C-m

# pane 2 (左下): SLAM 输出 odom 频率
tmux select-pane -t "$SESSION:monitor.0"
tmux split-window -t "$SESSION:monitor" -v
tmux send-keys -t "$SESSION:monitor" "$COMMON_INIT" C-m
tmux send-keys -t "$SESSION:monitor" \
    "echo '[lightning odom hz]' && sleep $((STACK_WARMUP_S + 5)) && ros2 topic hz /lightning/odom" C-m

# pane 3 (右下): TF echo on demand
tmux select-pane -t "$SESSION:monitor.2"
tmux split-window -t "$SESSION:monitor" -h
tmux send-keys -t "$SESSION:monitor" "$COMMON_INIT" C-m
tmux send-keys -t "$SESSION:monitor" \
    "echo '[shell] check TF: ros2 run tf2_ros tf2_echo radar_uper_Link radar_f_Link'" C-m

tmux select-layout -t "$SESSION:monitor" tiled

# ---------------------------------------------------------------------------
# Window 4: bag — 预填录制命令，不自动按 Enter
# ---------------------------------------------------------------------------
tmux new-window -t "$SESSION" -n "bag"
tmux send-keys -t "$SESSION:bag" "$COMMON_INIT" C-m
tmux send-keys -t "$SESSION:bag" \
    "mkdir -p $ROS2_WS/ros2_bags && cd $ROS2_WS/ros2_bags" C-m
tmux send-keys -t "$SESSION:bag" \
    "echo '--- press Enter to start recording, Ctrl+C to stop ---'" C-m
# 这一行不发 C-m，让用户手动按回车
# 同时录三路原始点云和原始 IMU，便于离线复现融合/单位转换链路
# --storage mcap     : 流式格式，多路点云高吞吐下不易丢消息，崩溃后可恢复
# --max-bag-size 4G  : 单文件 4GB 自动切分，便于传输 / 局部回放
tmux send-keys -t "$SESSION:bag" \
    "ros2 bag record --storage mcap --max-bag-size 4000000000 -o d100_$(date +%Y%m%d_%H%M%S) /merged/LIDAR/POINTS /LIDAR/POINTS /chin/LIDAR/POINTS /tail/LIDAR/POINTS /imu/data /rslidar_imu_data /tf /tf_static /joint_states /odom /lightning/odom"

# ---------------------------------------------------------------------------
# Window 5: shell — 空闲 shell（save_map / 临时命令）
# ---------------------------------------------------------------------------
tmux new-window -t "$SESSION" -n "shell"
tmux send-keys -t "$SESSION:shell" "$COMMON_INIT" C-m
tmux send-keys -t "$SESSION:shell" "clear" C-m
tmux send-keys -t "$SESSION:shell" \
    "echo '保存地图: ros2 service call /lightning/save_map lightning/srv/SaveMap \"{map_id: \\\"map_$(date +%Y%m%d)\\\"}\"'" C-m

# ---------------------------------------------------------------------------
# 默认聚焦到 stack 窗口
# ---------------------------------------------------------------------------
tmux select-window -t "$SESSION:stack"

# ---------------------------------------------------------------------------
# 进入 session（如果在 SSH 里执行，会直接 attach；脚本里执行则后台运行）
# ---------------------------------------------------------------------------
if [[ -t 1 ]]; then
    tmux attach -t "$SESSION"
else
    echo "session '$SESSION' started in background. attach with: tmux attach -t $SESSION"
fi
