#!/bin/bash
# ===========================================
# 에뮬레이터 연결 확인 후 컨트롤러 스폰 스크립트
# ===========================================

echo "=========================================="
echo " Waiting for emulator connection..."
echo "=========================================="

# ROS2 환경 설정
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

MAX_RETRIES=60  # 최대 60초 대기
RETRY_COUNT=0

# 에뮬레이터 연결 대기 (get_robot_state 서비스 응답 확인)
while [ $RETRY_COUNT -lt $MAX_RETRIES ]; do
    # 서비스가 존재하는지 확인
    if ros2 service list 2>/dev/null | grep -q "/dsr01/system/get_robot_state"; then
        # 서비스 호출 시도
        RESULT=$(timeout 3 ros2 service call /dsr01/system/get_robot_state dsr_msgs2/srv/GetRobotState '{}' 2>&1)

        if echo "$RESULT" | grep -q "success=True"; then
            echo ""
            echo "=========================================="
            echo " Emulator connected! (attempt $((RETRY_COUNT+1)))"
            echo "=========================================="
            break
        fi
    fi

    RETRY_COUNT=$((RETRY_COUNT+1))
    echo -n "."
    sleep 1
done

if [ $RETRY_COUNT -ge $MAX_RETRIES ]; then
    echo ""
    echo "ERROR: Emulator connection timeout after ${MAX_RETRIES} seconds"
    exit 1
fi

# 잠시 대기 (안정화)
echo "Waiting 2 seconds for stabilization..."
sleep 2

# 컨트롤러 스폰
echo ""
echo "=========================================="
echo " Spawning joint_state_broadcaster..."
echo "=========================================="
ros2 run controller_manager spawner joint_state_broadcaster \
    -c /dsr01/controller_manager \
    --controller-manager-timeout 30 &
SPAWN1_PID=$!

echo ""
echo "=========================================="
echo " Spawning dsr_controller2..."
echo "=========================================="
ros2 run controller_manager spawner dsr_controller2 \
    -c /dsr01/controller_manager \
    --controller-manager-timeout 30 &
SPAWN2_PID=$!

# 스폰 완료 대기
wait $SPAWN1_PID
SPAWN1_RESULT=$?
wait $SPAWN2_PID
SPAWN2_RESULT=$?

if [ $SPAWN1_RESULT -eq 0 ] && [ $SPAWN2_RESULT -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo " All controllers spawned successfully!"
    echo "=========================================="
else
    echo ""
    echo "WARNING: Some controllers may have failed to spawn"
    echo "  joint_state_broadcaster: exit code $SPAWN1_RESULT"
    echo "  dsr_controller2: exit code $SPAWN2_RESULT"
fi

exit 0
