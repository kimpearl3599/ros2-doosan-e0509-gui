#!/bin/bash
# 두산 E0509 로봇 GUI 시작 스크립트

echo "=== 두산 E0509 GUI 시작 ==="

docker exec -it ros2-doosan bash -c "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run my_ros2_assignment robot_gui"
