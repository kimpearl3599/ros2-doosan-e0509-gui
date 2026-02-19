#!/bin/bash
# 두산 E0509 로봇 시뮬레이션 시작 스크립트

echo "=== 두산 E0509 시뮬레이션 시작 ==="

# 1. ros2-doosan 컨테이너 재시작
echo "[1/4] Docker 컨테이너 재시작..."
docker restart ros2-doosan
sleep 5

# 2. 에뮬레이터 정리 (컨테이너 안에서)
echo "[2/4] 에뮬레이터 정리..."
docker exec ros2-doosan bash -c "docker stop dsr01_emulator 2>/dev/null; docker rm dsr01_emulator 2>/dev/null"
sleep 2

# 3. 시뮬레이션 실행
echo "[3/4] Gazebo 시뮬레이션 시작..."
echo ""
echo "=========================================="
echo "  Gazebo 창에서 play 버튼(▶)을 클릭하세요!"
echo "=========================================="
echo ""

docker exec -it ros2-doosan bash -c "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch dsr_bringup2 dsr_bringup2_gazebo.launch.py model:=e0509"
