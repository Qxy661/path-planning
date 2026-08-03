#!/bin/bash
# TurtleBot3 巡检导航 - SLAM 建图启动脚本
# 时序：Gazebo → 手动spawn机器人 → Cartographer建图
# 解决 launch 时序导致 /spawn_entity 不可用的问题

set -e
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger

echo "=== 1. 启动 Gazebo 世界 (后台) ==="
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py \
  > /tmp/tb3_gazebo.log 2>&1 &
GAZEBO_PID=$!
echo "Gazebo PID: $GAZEBO_PID"

echo "=== 2. 等待 /spawn_entity 服务就绪 ==="
for i in $(seq 1 20); do
  if ros2 service list 2>/dev/null | grep -q spawn_entity; then
    echo "服务就绪 (${i}x2s)"
    break
  fi
  sleep 2
done

echo "=== 3. 手动 spawn TurtleBot3 ==="
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity \
  "{name: burger, xml: \"$(cat /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf)\", \
   robot_namespace: '', \
   initial_pose: {position: {x: -2.0, y: -0.5, z: 0.01}, orientation: {w: 1.0}}}" \
  > /tmp/tb3_spawn.log 2>&1 || true
grep -i success /tmp/tb3_spawn.log && echo "机器人已加载" || echo "spawn需检查"

echo "=== 4. 启动 Cartographer 建图 (后台) ==="
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True \
  > /tmp/tb3_cartographer.log 2>&1 &
echo "Cartographer PID: $!"

echo "=== 5. 等待建图话题 ==="
sleep 20
echo "话题总数: $(ros2 topic list 2>/dev/null | wc -l)"
echo "scan: $(ros2 topic list 2>/dev/null | grep -c scan)"
echo "map: $(ros2 topic list 2>/dev/null | grep -c map)"

echo "=== SLAM 建图已启动 ==="
echo "下一步：控制小车移动建图 (发布 /cmd_vel)"
