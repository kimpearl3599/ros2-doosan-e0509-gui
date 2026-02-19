#!/bin/bash
# 두산 E0509 로봇 전체 시작 스크립트 (시뮬레이션 + GUI)

echo "=== 두산 E0509 전체 시작 ==="

# 1. ros2-doosan 컨테이너 재시작
echo "[1/5] Docker 컨테이너 재시작..."
docker restart ros2-doosan
sleep 5

# 2. 에뮬레이터 정리
echo "[2/5] 에뮬레이터 정리..."
docker exec ros2-doosan bash -c "docker stop dsr01_emulator 2>/dev/null; docker rm dsr01_emulator 2>/dev/null"
sleep 2

# 3. 시뮬레이션을 백그라운드로 시작
echo "[3/5] Gazebo 시뮬레이션 시작 (백그라운드)..."
gnome-terminal --title="Gazebo Simulation" -- bash -c "docker exec -it ros2-doosan bash -c 'source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch dsr_bringup2 dsr_bringup2_gazebo.launch.py model:=e0509'; exec bash" 2>/dev/null || \
xterm -T "Gazebo Simulation" -e "docker exec -it ros2-doosan bash -c 'source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch dsr_bringup2 dsr_bringup2_gazebo.launch.py model:=e0509'; exec bash" 2>/dev/null || \
echo "새 터미널을 열 수 없습니다. 수동으로 start_simulation.sh를 실행하세요."

echo ""
echo "=========================================="
echo "  Gazebo 창에서 play 버튼(▶)을 클릭하세요!"
echo "  그 후 Enter를 눌러 GUI를 시작합니다."
echo "=========================================="
read -p "Press Enter to start GUI..."

# 4. GUI 시작
echo "[4/5] GUI 시작..."
docker exec -it ros2-doosan bash -c "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run my_ros2_assignment robot_gui"
